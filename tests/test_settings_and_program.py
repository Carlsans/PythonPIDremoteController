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
        if reached82at is None and program.currentstage == 1:
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


if __name__ == '__main__':
    test_settingsstore()
    test_greek_yogurt_program()
    print("\nOK - all settings/program tests passed.")
