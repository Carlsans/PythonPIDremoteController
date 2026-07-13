"""Reproduces the real 38 C overshoot (8.3 C measured on hardware 2026-07-13)
and proves the setpoint-ramp fix.

Hardware telemetry showed the PID cut its output BEFORE 38 C was reached and
the water still climbed to 46.3 C: heat stored in the heating element kept
flowing in for ~10 minutes. No PID tuning can see that stored heat, so the
approach must give it time to arrive: full power only until `rampzone` below
the target, then the setpoint ramps up at `ramprate` (PIDProgram).

The TwoCapacityPot (element + water) is fitted to the measured failure and to
the measured full-power climb rate; the MCU model reproduces the firmware's
agg/cons switching at a 4.5 C gap and its 160/255 output clamp.

Run from the project root:
    ./venvarch/bin/python tests/test_overshoot_fix.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
os.environ["YOGURT_SILENT"] = "1"

from src.PIDProgram import PIDProgram
from tests.simulator import TwoCapacityPot, SimulatedAdaptiveMCUPID, FakeController

# water-82c profile (ZN-PI from the real autotune)
PROFILE = (15.61059978219356, 0.02244694864025816, 0.0)


def runstage(target, startattemp, rampzone, simhours=3, elemtemp=None, oldallpi=False):
    pot = TwoCapacityPot()
    pot.temp = startattemp
    pot.elemtemp = startattemp if elemtemp is None else elemtemp
    pid = SimulatedAdaptiveMCUPID()
    controller = FakeController(pid)
    controller.currenttemp = startattemp
    simtime = {"t": 0.0}
    program = PIDProgram(controller,
                         stages=[{"temperature": target, "duration_minutes": 60.0}],
                         tunings=PROFILE, timesource=lambda: simtime["t"],
                         rampzone=rampzone)
    if oldallpi:
        # What the code did before the cons/agg fix: PI in both slots, so the
        # integral winds up over the whole climb.
        controller.setAllPID(PROFILE[0], PROFILE[1], PROFILE[2])

    overshoot = 0.0
    reachedat = None
    finalerrors = []
    total = simhours * 60 * 60
    for t in range(total):
        simtime["t"] = float(t)
        pid.setpoint = controller.SP
        measured = round(pot.temp * 4) / 4.0
        output = pid.compute(measured)
        pot.step(output)
        controller.currenttemp = round(pot.temp * 4) / 4.0
        program.applyProgram()
        if reachedat is None and pot.temp >= target:
            reachedat = t
        overshoot = max(overshoot, pot.temp - target)
        if t >= total - 30 * 60:
            finalerrors.append(abs(pot.temp - target))
    assert reachedat is not None, "never reached " + str(target)
    return overshoot, max(finalerrors), reachedat


def main():
    # 1. Reproduce the user's original 'massive overshoot': the pre-fix code
    #    (PI in both firmware slots -> integral windup) plus the pot's stored
    #    heat, cold start 22 -> 38 C.
    overshootold, _, _ = runstage(38.0, startattemp=22.0, rampzone=0.0, oldallpi=True)
    print("OLD code (all-PI, no ramp) 22->38 C: overshoot = "
          + str(round(overshootold, 2)) + " C")
    assert overshootold > 3.0, \
        "model no longer reproduces the reported overshoot: " + str(overshootold)

    # 1b. cons/agg fix alone (no ramp): stored heat still overshoots.
    overshootdirect, _, _ = runstage(38.0, startattemp=22.0, rampzone=0.0)
    print("cons/agg fix only (no ramp) 22->38 C: overshoot = "
          + str(round(overshootdirect, 2)) + " C (hardware showed this can reach 8.3"
          + " with a residual-hot plate)")

    # 2. The ramp fix, same cold start.
    overshoot38, finalerr38, reached38 = runstage(38.0, startattemp=22.0, rampzone=10.0)
    print("Ramped approach 22->38 C: overshoot = " + str(round(overshoot38, 2))
          + " C, reached in " + str(round(reached38 / 60.0, 1))
          + " min, max |error| last 30 min = " + str(round(finalerr38, 3)) + " C")
    assert overshoot38 < 1.5, "ramp fix insufficient at 38 C: " + str(overshoot38)
    assert reached38 < 60 * 60, "ramped approach too slow: " + str(reached38) + "s"
    assert finalerr38 < 0.5, "does not hold 38 C: " + str(finalerr38)

    # 2b. Today's hardware condition: water 33 C but the plate still hot from
    #     a previous 82 C run. The ramp cannot remove heat that is already
    #     stored (no active cooling exists), but it must not add to it.
    overshootres, finalerrres, _ = runstage(38.0, startattemp=33.0, rampzone=10.0,
                                            elemtemp=60.0)
    print("Ramped approach 33->38 C with residual-hot plate: overshoot = "
          + str(round(overshootres, 2)) + " C, max |error| last 30 min = "
          + str(round(finalerrres, 3)) + " C")
    assert overshootres < 3.0, "ramp should limit residual-heat overshoot: " + str(overshootres)
    assert finalerrres < 0.5, "does not settle at 38 C: " + str(finalerrres)

    # 3. The ramp must not break the 82 C stage (long climb from cold).
    overshoot82, finalerr82, reached82 = runstage(82.0, startattemp=22.0, rampzone=10.0,
                                                  simhours=4)
    print("Ramped approach at 82 C: overshoot = " + str(round(overshoot82, 2))
          + " C, reached in " + str(round(reached82 / 60.0, 1))
          + " min, max |error| last 30 min = " + str(round(finalerr82, 3)) + " C")
    assert overshoot82 < 2.0, "ramp regressed the 82 C stage: " + str(overshoot82)
    assert reached82 < 100 * 60, "82 C approach too slow: " + str(reached82) + "s"
    assert finalerr82 < 0.6, "does not hold 82 C: " + str(finalerr82)

    print("\nOK - stored-heat overshoot reproduced without ramp, fixed with ramp.")


if __name__ == '__main__':
    main()
