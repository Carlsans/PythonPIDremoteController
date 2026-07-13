"""Tests for SettingsStore and the staged PIDProgram (Greek-yogurt scenario).

Runs a full simulated Greek-yogurt program: heat to 82 C (texture/sanitizing),
hold 10 minutes, then drop to 40 C and hold - all in simulated time (instant).

Run from the project root:
    ./venvarch/bin/python tests/test_settings_and_program.py
"""

import json
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
os.environ["YOGURT_SILENT"] = "1"  # no chime sounds during tests

from src.SettingsStore import SettingsStore, DEFAULTSETTINGS
from src.PIDProgram import PIDProgram
from tests.simulator import SimulatedPot, SimulatedMCUPID, FakeController


def test_settingsstore():
    with tempfile.TemporaryDirectory() as tmpdir:
        path = os.path.join(tmpdir, "settings.json")
        store = SettingsStore(path=path)

        # Missing file -> defaults.
        settings = store.load()
        assert settings == DEFAULTSETTINGS
        assert settings is not DEFAULTSETTINGS and settings["stages"] is not DEFAULTSETTINGS["stages"], \
            "load() must return a copy, not the shared defaults"

        # Roundtrip.
        settings["stages"] = [{"temperature": 82.0, "duration_minutes": 10.0},
                              {"temperature": 40.0, "duration_minutes": 8 * 60.0}]
        settings["pid_profiles"]["water-82c"] = {"Kp": 50.0, "Ki": 0.1, "Kd": 0.0}
        settings["active_profile"] = "water-82c"
        store.save(settings)
        reloaded = SettingsStore(path=path).load()
        assert reloaded == settings, "roundtrip mismatch"
        with open(path) as f:
            assert json.load(f)["active_profile"] == "water-82c"

        # Corrupt file -> defaults, no crash.
        with open(path, "w") as f:
            f.write("{ not json")
        assert SettingsStore(path=path).load() == DEFAULTSETTINGS
    print("OK - SettingsStore")


def test_greek_yogurt_program():
    pot = SimulatedPot(ambient=22.0, fullpowerrise=90.0, tau=1500.0, deadtime=20.0)
    pid = SimulatedMCUPID()
    controller = FakeController(pid)
    simtime = {"t": 0.0}
    stages = [{"temperature": 82.0, "duration_minutes": 10.0},
              {"temperature": 40.0, "duration_minutes": 60.0}]
    program = PIDProgram(controller, stages=stages, tunings=(81.2, 0.255, 0.0),
                         timesource=lambda: simtime["t"])
    assert controller.SP == 82.0, "first stage temperature not set"
    assert ("cons", 81.2, 0.255, 0.0) in controller.pidcalls, "profile not applied to cons tunings"
    assert ("agg", 100.0, 0.0, 0.0) in controller.pidcalls, "no-windup approach tunings not applied to agg"

    reached82at = None
    sp40at = None
    maxovershootat82 = 0.0
    maxerrorsettled82 = 0.0
    finalerrors = []
    totalseconds = 4 * 60 * 60
    for t in range(totalseconds):
        simtime["t"] = float(t)
        pid.setpoint = controller.SP  # resync loop does this in the real system
        output = pid.compute(round(pot.temp * 4) / 4.0)
        pot.step(output)
        controller.currenttemp = round(pot.temp * 4) / 4.0
        program.applyProgram()
        if reached82at is None and program.temperaturehasbeenreached:
            reached82at = t
        if reached82at is not None and controller.SP == 82.0:
            maxovershootat82 = max(maxovershootat82, pot.temp - 82.0)
            if t > reached82at + 300:  # settled part of the hold
                maxerrorsettled82 = max(maxerrorsettled82, abs(pot.temp - 82.0))
        if sp40at is None and controller.SP == 40.0:
            sp40at = t
        if t >= totalseconds - 15 * 60:
            finalerrors.append(abs(pot.temp - 40.0))

    assert reached82at is not None, "82 C was never reached"
    assert sp40at is not None, "program never moved to the 40 C stage"
    holdlength = sp40at - reached82at
    assert abs(holdlength - 10 * 60) < 30, \
        "82 C hold was " + str(holdlength) + "s, expected ~600s"
    # ~1 C of approach overshoot comes from MCU integral windup on the long
    # full-power climb; what matters for texture/sanitizing is a bounded
    # overshoot and a tight hold once settled.
    assert maxovershootat82 < 2.5, \
        "overshoot at 82 C too large: " + str(maxovershootat82) + " C"
    assert maxerrorsettled82 < 0.6, \
        "settled hold at 82 C too loose: " + str(maxerrorsettled82) + " C"
    assert program.currentstage == 2, "program did not run through both stages"
    assert max(finalerrors) < 0.5, \
        "does not hold 40 C at the end: " + str(max(finalerrors)) + " C"
    print("OK - Greek-yogurt program: 82 C reached at t=" + str(reached82at)
          + "s, overshoot " + str(round(maxovershootat82, 2))
          + " C, settled hold within " + str(round(maxerrorsettled82, 3))
          + " C, then settled at 40 C (max error last 15 min: "
          + str(round(max(finalerrors), 3)) + " C)")


def test_progress_reporting():
    controller = FakeController(SimulatedMCUPID())
    simtime = {"t": 0.0}
    stages = [{"temperature": 50.0, "duration_minutes": 5.0},
              {"temperature": 30.0, "duration_minutes": 2.0}]
    program = PIDProgram(controller, stages=stages, tunings=(10, 0.01, 0.0),
                         timesource=lambda: simtime["t"])

    # Heating: stage 0, no hold time consumed yet, total remaining is the
    # sum of BOTH stages' hold durations (heating time is not predictable
    # and intentionally excluded).
    controller.currenttemp = 40.0
    simtime["t"] = 10.0
    program.applyProgram()
    progress = program.getprogress()
    assert progress == {
        "ended": False, "phase": "heating", "stage_index": 0, "stage_count": 2,
        "target": 50.0, "total_elapsed_s": 10.0, "stage_elapsed_s": 0.0,
        "stage_remaining_s": 5 * 60.0, "total_remaining_s": 5 * 60.0 + 2 * 60.0,
    }, progress

    # Reach the target: the hold begins. currentstage must NOT advance yet -
    # this is exactly the bug that used to make the program report "ended"
    # (and skip the final stage's completion chime) the instant its target
    # was reached, without ever honoring its hold duration.
    controller.currenttemp = 50.0
    simtime["t"] = 11.0
    program.applyProgram()
    assert program.temperaturehasbeenreached is True
    assert program.currentstage == 0, "currentstage must not advance until the hold finishes"

    simtime["t"] = 11.0 + 60.0  # one minute into the 5-minute hold
    program.applyProgram()
    progress = program.getprogress()
    assert progress["phase"] == "holding"
    assert progress["stage_index"] == 0
    assert abs(progress["stage_elapsed_s"] - 60.0) < 1e-6
    assert abs(progress["stage_remaining_s"] - (5 * 60 - 60.0)) < 1e-6
    assert abs(progress["stage_elapsed_s"] + progress["stage_remaining_s"] - 5 * 60) < 1e-9
    assert abs(progress["total_remaining_s"] - (progress["stage_remaining_s"] + 2 * 60)) < 1e-9

    # Hold finishes: advances to stage 1 and applies its target.
    simtime["t"] = 11.0 + 5 * 60 + 1
    program.applyProgram()
    assert program.currentstage == 1
    assert controller.SP == 30.0
    progress = program.getprogress()
    assert progress["ended"] is False
    assert progress["stage_index"] == 1
    assert progress["target"] == 30.0
    assert progress["total_remaining_s"] == 2 * 60.0  # only stage 1's hold left

    # Stage 1 (the LAST stage) reaches its target and holds for its full
    # duration before the program is considered ended - not immediately.
    controller.currenttemp = 30.0
    simtime["t"] += 1
    program.applyProgram()
    assert program.temperaturehasbeenreached is True
    progress = program.getprogress()
    assert progress["ended"] is False, \
        "must not report ended before the last stage's hold duration elapses"
    assert progress["phase"] == "holding" and progress["stage_index"] == 1

    simtime["t"] += 2 * 60 + 1
    program.applyProgram()
    progress = program.getprogress()
    assert progress["ended"] is True
    assert progress["target"] == 30.0
    assert progress["total_remaining_s"] == 0.0
    print("OK - PIDProgram.getprogress() (and the last-stage hold-duration fix)")


if __name__ == '__main__':
    test_settingsstore()
    test_greek_yogurt_program()
    test_progress_reporting()
    print("\nOK - all settings/program tests passed.")
