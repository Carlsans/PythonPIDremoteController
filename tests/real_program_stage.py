"""REAL-HARDWARE test (water only!): run one staged-program stage and
measure approach overshoot + hold quality, then turn the heater off.

Target and hold come from env vars:
    YOGURT_TEST_TARGET        (default 38)
    YOGURT_TEST_HOLD_MINUTES  (default 10)
    YOGURT_TEST_PROFILE       (default: the settings' active_profile)

Run from the project root:
    MPLBACKEND=Agg PYTHONUNBUFFERED=1 ./venvarch/bin/python tests/real_program_stage.py
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
os.environ.setdefault("MPLBACKEND", "Agg")
os.environ.setdefault("YOGURT_SILENT", "1")

from src.SettingsStore import SettingsStore
from src.yogurtdata import YogourtFermenter

TARGET = float(os.environ.get("YOGURT_TEST_TARGET", 38))
HOLDMINUTES = float(os.environ.get("YOGURT_TEST_HOLD_MINUTES", 10))
HARDTIMEOUT = 2 * 60 * 60

settings = SettingsStore().load()
profilename = os.environ.get("YOGURT_TEST_PROFILE", settings["active_profile"])
profile = settings["pid_profiles"][profilename]
print("Using profile " + profilename + ":", profile, "- target", TARGET, "C hold", HOLDMINUTES, "min")

state = {"fermenter": None, "reachedat": None, "temps": [], "laststatus": 0.0,
         "starttime": time.time(), "maxtemp": 0.0}


def ontick():
    fermenter = state["fermenter"]
    now = time.time()
    if fermenter.currenttemp > state["maxtemp"]:
        state["maxtemp"] = fermenter.currenttemp
    if now - state["laststatus"] >= 60:
        state["laststatus"] = now
        print("STATUS: temp=" + str(fermenter.currenttemp) + " SP=" + str(fermenter.currentSP)
              + " CV=" + str(fermenter.currentCV)
              + " elapsed=" + str(int(now - state["starttime"])) + "s")
    if state["reachedat"] is None and fermenter.pidprogram.temperaturehasbeenreached:
        state["reachedat"] = now
        print("REACHED " + str(TARGET) + "C after " + str(int(now - state["starttime"])) + "s - measuring hold")
    if state["reachedat"] is not None:
        state["temps"].append(fermenter.currenttemp)
        if now - state["reachedat"] >= HOLDMINUTES * 60:
            finish("hold measured")
    if now - state["starttime"] > HARDTIMEOUT:
        finish("HARD TIMEOUT")


def finish(reason):
    fermenter = state["fermenter"]
    print("FINISHING:", reason, "- turning heater off (SetSP(1))")
    fermenter.setSP(1)
    fermenter.stoprequested = True


def main():
    print("Starting REAL staged program (water only).")
    state["fermenter"] = YogourtFermenter(
        mode='pidprogram',
        stages=[{"temperature": TARGET, "duration_minutes": HOLDMINUTES}],
        tunings=(profile["Kp"], profile["Ki"], profile["Kd"]),
        ontick=ontick, autorun=False)
    state["fermenter"].listeningloop()

    temps = [t for t in state["temps"] if t > 0]
    if not temps:
        print("RESULT: FAILED - no hold data")
        sys.exit(1)
    # Overshoot is judged on the maximum temperature ever seen (including
    # any late peak after the hold window started).
    overshoot = state["maxtemp"] - TARGET
    settled = temps[len(temps) // 2:]
    settlederrs = [abs(t - TARGET) for t in settled]
    print("RESULT: " + str(TARGET) + "C stage -> overshoot=" + str(round(overshoot, 2))
          + "C settled max|err|=" + str(round(max(settlederrs), 2))
          + "C settled mean|err|=" + str(round(sum(settlederrs) / len(settlederrs), 2))
          + "C (n=" + str(len(temps)) + ")")
    print("RESULT: OK - heater is now OFF (SP=1).")


if __name__ == '__main__':
    main()
