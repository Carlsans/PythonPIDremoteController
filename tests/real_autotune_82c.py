"""REAL-HARDWARE test (water only!): relay autotune at 82 C on the actual ESP.

Runs the relay autotune at 82 C, saves the resulting tunings as the PID
profile 'water-82c' in yogurt_settings.json (the same path the GUI uses),
then verifies a 10-minute hold at 82 C with the new tunings and finally
turns the heater off (SetSP(1)).

This script talks to the real device. Only run it when the pot contains
water. Run from the project root:
    MPLBACKEND=Agg PYTHONUNBUFFERED=1 ./venvarch/bin/python tests/real_autotune_82c.py
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
os.environ.setdefault("MPLBACKEND", "Agg")

from src.SettingsStore import SettingsStore
from src.yogurtdata import YogourtFermenter

TARGET = 82.0
PROFILELABEL = "water-82c"
HOLDSECONDS = 10 * 60
HARDTIMEOUT = 3 * 60 * 60  # abort everything after 3 h no matter what

state = {"fermenter": None, "holdstart": None, "holdtemps": [],
         "laststatus": 0.0, "failed": None, "starttime": time.time()}


def ondone(result):
    if result is None:
        state["failed"] = "autotune aborted"
        return
    store = SettingsStore()
    settings = store.load()
    tunings = result[result.get("applied", "ziegler_nichols_pi")]
    settings["pid_profiles"][PROFILELABEL] = {
        "Kp": tunings["Kp"], "Ki": tunings["Ki"], "Kd": tunings["Kd"]}
    settings["active_profile"] = PROFILELABEL
    store.save(settings)
    print("PROFILE SAVED:", PROFILELABEL, settings["pid_profiles"][PROFILELABEL])
    print("HOLD PHASE: verifying " + str(HOLDSECONDS) + "s hold at", TARGET)
    state["holdstart"] = time.time()


def ontick():
    fermenter = state["fermenter"]
    now = time.time()
    if now - state["laststatus"] >= 60:
        state["laststatus"] = now
        print("STATUS: temp=" + str(fermenter.currenttemp) + " SP=" + str(fermenter.currentSP)
              + " CV=" + str(fermenter.currentCV) + " tuner=" + fermenter.relayautotune.state
              + " elapsed=" + str(int(now - state["starttime"])) + "s")
    if state["holdstart"] is not None:
        state["holdtemps"].append(fermenter.currenttemp)
        if now - state["holdstart"] >= HOLDSECONDS:
            finish("hold complete")
    if now - state["starttime"] > HARDTIMEOUT:
        state["failed"] = "hard timeout"
        finish("HARD TIMEOUT - aborting")
    if state["failed"] == "autotune aborted":
        finish("autotune aborted")


def finish(reason):
    fermenter = state["fermenter"]
    print("FINISHING:", reason, "- turning heater off (SetSP(1))")
    fermenter.setSP(1)
    fermenter.stoprequested = True


def main():
    print("Starting REAL autotune at", TARGET, "C (water only). Heater off at the end.")
    state["fermenter"] = YogourtFermenter(mode='relayautotune', autotunetarget=TARGET,
                                          onautotunedone=ondone, ontick=ontick,
                                          autorun=False)
    state["fermenter"].listeningloop()

    holdtemps = [t for t in state["holdtemps"] if t > 0]
    if state["failed"]:
        print("RESULT: FAILED -", state["failed"])
        sys.exit(1)
    if holdtemps:
        # Skip the first 5 min (approach/overshoot), judge the settled half.
        settled = holdtemps[len(holdtemps) // 2:]
        maxerr = max(abs(t - TARGET) for t in settled)
        overshoot = max(holdtemps) - TARGET
        print("RESULT: hold at 82C -> overshoot=" + str(round(overshoot, 2))
              + "C settled max error=" + str(round(maxerr, 2))
              + "C (n=" + str(len(holdtemps)) + " samples)")
        print("RESULT: OK - heater is now OFF (SP=1). Profile '" + PROFILELABEL + "' saved.")
    else:
        print("RESULT: no hold data collected")


if __name__ == '__main__':
    main()
