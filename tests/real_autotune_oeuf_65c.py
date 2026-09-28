"""REAL-HARDWARE run: relay autotune at 65 C ("oeuf 65C" - egg, not yogurt)
on the actual ESP. Saves the resulting tunings as the PID profile
'oeuf 65C' in yogurt_settings.json (the same file the GUI reads/writes),
whether the run completes fully or is stopped/times out early - see
RelayAutotune.abort()'s partial-tuning support.

This script talks to the real device (192.168.0.4:5000 by default). Only
run it with the pot safely prepared (water, or whatever is actually going
to be heated) and someone able to check on it. Heater is forced off
(SetSP(1)) whenever the run ends, however it ends.

Run from the project root:
    MPLBACKEND=Agg PYTHONUNBUFFERED=1 ./venvarch/bin/python tests/real_autotune_oeuf_65c.py

Every line printed is also mirrored to a timestamped file under logs/ (see
YogourtFermenter.runlogpath) - check that file if this terminal's
scrollback is ever lost; a run like this can take a long time.
"""

import os
import signal
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
os.environ.setdefault("MPLBACKEND", "Agg")

from src.SettingsStore import SettingsStore
from src.NetworkConfiguration import NetworkConfiguration
from src.yogurtdata import YogourtFermenter

TARGET = float(os.environ.get("YOGURT_REAL_TARGET", 65.0))
PROFILELABEL = os.environ.get("YOGURT_REAL_PROFILE", "oeuf 65C")
# The relay's own "heater on" setpoint is target+15 (see RelayAutotune.relay()),
# capped at maxsafetemp = target+SAFETYMARGIN. yogurtdata.py's own GUI default
# margin is 12, which is BELOW 15 - meaning maxsafetemp always ends up exactly
# equal to the relay's own on-setpoint, leaving zero headroom before a normal
# oscillation peak trips the hard safety abort in RelayAutotune.update(). That
# went unnoticed on the proven 82 C profile because that pot's real thermal
# time constant (~1500s) is huge next to the 1s sample loop, so overshoot per
# sample is tiny - but 65 C is new, unproven territory, so this uses a larger
# margin to leave a few real degrees of headroom instead of pinning exactly at
# the ceiling (validated in scratch testing: a fast/twitchy pot with margin=12
# clipped a normal oscillation peak and triggered a spurious safety abort).
SAFETYMARGIN = float(os.environ.get("YOGURT_REAL_SAFETY_MARGIN", 20.0))
# Belt-and-suspenders ceiling independent of RelayAutotune's own
# timeoutseconds: that clock only starts once the tuner gets its first real
# temperature reading (state leaves 'init'), so it cannot catch an ESP that
# never answers at all - only this script-level timer can.
HARDTIMEOUT = float(os.environ.get("YOGURT_REAL_HARD_TIMEOUT", 3 * 60 * 60))
FIRSTTEMPTIMEOUT = float(os.environ.get("YOGURT_REAL_FIRST_TEMP_TIMEOUT", 60))
STATUSINTERVAL = 30

state = {"fermenter": None, "laststatus": 0.0, "starttime": time.time(),
         "saved": False, "firsttempseen": False, "stoprequested_reason": None}


def ondone(result):
    state["saved"] = True
    if result is None:
        print("RESULT: aborted, no usable oscillation data was measured - nothing saved.")
        return
    store = SettingsStore()
    settings = store.load()
    tunings = result[result.get("applied", "ziegler_nichols_pi")]
    settings["pid_profiles"][PROFILELABEL] = {
        "Kp": tunings["Kp"], "Ki": tunings["Ki"], "Kd": tunings["Kd"]}
    settings["active_profile"] = PROFILELABEL
    store.save(settings)
    print("PROFILE SAVED:", PROFILELABEL, settings["pid_profiles"][PROFILELABEL])
    tag = "PARTIAL (" + str(result.get("aborted_reason")) + ")" if result.get("aborted_reason") else "FULL"
    print("RESULT:", tag, "- cycles_used=" + str(result.get("cycles_used")) + "/"
          + str(result.get("cycles_target")))
    print("RESULT: Ku=" + str(result["Ku"]) + " Tu=" + str(result["Tu"]) + "s amplitude=" + str(result["amplitude"]))
    print("RESULT: applied tunings (ziegler_nichols_pi) =", result["ziegler_nichols_pi"])


def requeststop(reason):
    if state["stoprequested_reason"] is not None:
        return  # already requested - avoid spamming this every tick until the loop actually exits
    state["stoprequested_reason"] = reason
    fermenter = state["fermenter"]
    if fermenter is None:
        return
    print("STOP REQUESTED:", reason)
    tuner = getattr(fermenter, "relayautotune", None)
    if tuner is not None and tuner.state == "relay":
        tuner.abort(reason)
    fermenter.setSP(1)
    fermenter.stoprequested = True


def handlesignal(signum, frame):
    requeststop("signal " + str(signum))


def ontick():
    fermenter = state["fermenter"]
    now = time.time()
    if fermenter.currenttemp > 0:
        state["firsttempseen"] = True
    if now - state["laststatus"] >= STATUSINTERVAL:
        state["laststatus"] = now
        tuner = getattr(fermenter, "relayautotune", None)
        print("STATUS: temp=" + str(fermenter.currenttemp) + " SP=" + str(fermenter.currentSP)
              + " CV=" + str(fermenter.currentCV)
              + " tuner=" + (tuner.state if tuner else "?")
              + " cycles=" + (str(len(tuner.cyclepeaks)) + "/" + str(tuner.skipcycles + tuner.cyclestomeasure)
                              if tuner else "?")
              + " elapsed=" + str(int(now - state["starttime"])) + "s")
    if not state["firsttempseen"] and now - state["starttime"] > FIRSTTEMPTIMEOUT:
        requeststop("no temperature reading received within " + str(FIRSTTEMPTIMEOUT)
                    + "s - the ESP8266 is likely unreachable")
    if now - state["starttime"] > HARDTIMEOUT:
        requeststop("script-level hard timeout after " + str(HARDTIMEOUT) + "s")


def preflightcheck():
    """Quick network-layer reachability check before committing to a
    possibly hours-long run, so a powered-off/unreachable device fails fast
    and loud instead of silently stalling (see FIRSTTEMPTIMEOUT, which is
    the real safety net - this is just a faster, clearer first signal)."""
    conf = NetworkConfiguration()
    print("PREFLIGHT: pinging", conf.esp8266_ip, "...")
    try:
        r = subprocess.run(["ping", "-c", "2", "-W", "2", conf.esp8266_ip],
                           capture_output=True, text=True, timeout=10)
        if r.returncode == 0:
            print("PREFLIGHT: OK - host is reachable on the network.")
            return True
        print("PREFLIGHT WARNING: ping to " + conf.esp8266_ip + " failed - the device may be off "
              "or unreachable. Continuing anyway; this will time out after " + str(FIRSTTEMPTIMEOUT)
              + "s if no data ever arrives.")
        return False
    except Exception as e:
        print("PREFLIGHT: ping check skipped (" + repr(e) + ")")
        return False


def main():
    print("Starting REAL relay autotune '" + PROFILELABEL + "' at", TARGET,
          "C. Heater is forced off whenever this run ends.")
    preflightcheck()
    state["fermenter"] = YogourtFermenter(mode='relayautotune', autotunetarget=TARGET,
                                          autotunesafetymargin=SAFETYMARGIN,
                                          onautotunedone=ondone, ontick=ontick,
                                          autorun=False)
    signal.signal(signal.SIGTERM, handlesignal)
    signal.signal(signal.SIGINT, handlesignal)
    try:
        state["fermenter"].listeningloop()
    finally:
        print("RUN LOG:", state["fermenter"].runlogpath)
    if not state["saved"]:
        print("RESULT: loop ended without the autotune ever finishing or being stopped cleanly.")
        sys.exit(1)


if __name__ == '__main__':
    main()
