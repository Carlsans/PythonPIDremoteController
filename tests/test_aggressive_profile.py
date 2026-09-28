"""Regression test for PIDProgram.updateaggressiveprofile().

The firmware switches between its 'agg' and 'cons' PID slots based on
|SP - temp| alone (~4.5 C on this hardware), so an overshoot past target by
more than that also lands on the bare Kp=100/Ki=0 approach profile - not
just an undershoot still climbing. This verifies that once the temperature
is within `aggressiveband` of target or above it, the agg slot is kept
mirroring the cons (tuned) profile instead, and that the true Kp=100
approach profile only loads while genuinely still climbing from well below.

Run from the project root:
    ./venvarch/bin/python tests/test_aggressive_profile.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.PIDProgram import PIDProgram
from tests.simulator import FakeController, SimulatedMCUPID


def lastaggcall(controller):
    aggcalls = [c for c in controller.pidcalls if c[0] == "agg"]
    assert aggcalls, "setaggPIDvalues was never called"
    return aggcalls[-1][1:]


def test_matches_cons_within_band_and_over_target():
    controller = FakeController(SimulatedMCUPID())
    tunings = (45.5, 0.0428, 0.0)
    program = PIDProgram(controller, stages=[{"temperature": 65.0, "duration_minutes": 60}],
                         tunings=tunings, aggressiveband=5.0)
    assert lastaggcall(controller) == (100.0, 0.0, 0.0), "should start with the aggressive approach profile"

    # Well below target - band (65 - 5 = 60): stays aggressive, no redundant resend.
    controller.currenttemp = 30.0
    callsbefore = len(controller.pidcalls)
    program.updateaggressiveprofile()
    assert len(controller.pidcalls) == callsbefore, "must not resend an unchanged aggressive profile"
    assert lastaggcall(controller) == (100.0, 0.0, 0.0)

    # Within the band (62 >= 60): agg must now mirror cons, not stay at 100.
    controller.currenttemp = 62.0
    program.updateaggressiveprofile()
    assert lastaggcall(controller) == tunings, \
        "agg should mirror the tuned cons profile once within aggressiveband of target"

    # Idempotent: calling again with the same temp/band must not resend.
    callsbefore = len(controller.pidcalls)
    program.updateaggressiveprofile()
    assert len(controller.pidcalls) == callsbefore, "must not resend an already-matched profile"

    # Over target: this is exactly the reported bug - Kp must NOT jump to 100.
    controller.currenttemp = 70.0
    program.updateaggressiveprofile()
    assert lastaggcall(controller) == tunings, \
        "agg must still mirror cons (not the bare Kp=100 profile) when over target"

    # Drops back well below target - band: the real aggressive climb profile
    # must return (e.g. after a big disturbance or a new, much higher stage).
    controller.currenttemp = 50.0
    program.updateaggressiveprofile()
    assert lastaggcall(controller) == (100.0, 0.0, 0.0), \
        "should return to the aggressive approach profile once genuinely far below target again"
    print("OK - agg mirrors cons within band and over target; true Kp=100 only while far below")


def test_follows_stage_transitions():
    controller = FakeController(SimulatedMCUPID())
    stagetunings = [(45.5, 0.0428, 0.0), (20.0, 0.02, 0.4)]
    simtime = {"t": 0.0}
    program = PIDProgram(controller,
                         stages=[{"temperature": 65.0, "duration_minutes": 0.0, "fast_approach": True},
                                 {"temperature": 39.0, "duration_minutes": 60}],
                         stagetunings=stagetunings, aggressiveband=5.0,
                         timesource=lambda: simtime["t"])

    # Sit within band of the first stage's target - agg should match stage 1's tunings.
    controller.currenttemp = 63.0
    program.updateaggressiveprofile()
    assert lastaggcall(controller) == stagetunings[0]

    # Reach stage 1's target (fast_approach sends it directly) and let its
    # zero-length hold elapse, advancing to stage 2.
    controller.currenttemp = 65.0
    program.applyProgram()
    simtime["t"] += 1.0
    program.applyProgram()
    assert program.currentstage == 1, "test setup did not advance to stage 2"

    # Still at 65 C, now measured against the NEW, much lower target (39):
    # 65 >= 39 - 5 = 34, i.e. "over range" of the new target. agg must
    # mirror the NEW stage's tunings, not stage 1's, and not 100.
    controller.currenttemp = 65.0
    program.updateaggressiveprofile()
    assert lastaggcall(controller) == stagetunings[1], \
        "agg must follow the new stage's cons profile after a stage transition"
    print("OK - agg tracks cons across stage transitions")


def main():
    test_matches_cons_within_band_and_over_target()
    test_follows_stage_transitions()
    print("\nOK - aggressive-profile band behaviour verified.")


if __name__ == '__main__':
    main()
