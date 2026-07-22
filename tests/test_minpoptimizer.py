"""Tests for MinPOptimizer: the "minimize P, then ratchet up I until it
holds" tuner, built after the real fermenter run showed PIDOptimizer's IAE
gradient descent never actually converging (the same Kp scored wildly
different IAE in different windows - real process noise, not the tuning,
was driving most of the variation).

Uses TwoCapacityPot (fitted to the REAL pot's measured 38 C stored-heat
overshoot behaviour) and SimulatedAdaptiveMCUPID (firmware agg/cons
gap-switching), driven for many simulated hours. As with PIDOptimizer, the
single most important property under test is the hard safety guarantee:
the measured temperature must NEVER exceed the configured maxswing from
setpoint once engaged.

Run from the project root:
    ./venvarch/bin/python tests/test_minpoptimizer.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.MinPOptimizer import MinPOptimizer
from tests.simulator import TwoCapacityPot, SimulatedAdaptiveMCUPID, FakeController

SETPOINT = 38.0
MAXSWING = 3.0


def makewarmedpot():
    pot = TwoCapacityPot()
    pot.temp = SETPOINT
    pot.elemtemp = SETPOINT + 2.0
    return pot


def makerig(starttunings, **optimizerkwargs):
    simtime = {"t": 0.0}
    pot = makewarmedpot()
    pid = SimulatedAdaptiveMCUPID()
    controller = FakeController(pid)
    controller.setSP(SETPOINT)
    controller.setaggPIDvalues(100.0, 0.0, 0.0)
    optimizer = MinPOptimizer(controller, starttunings, timesource=lambda: simtime["t"],
                              **optimizerkwargs)
    return simtime, pot, pid, controller, optimizer


def rundriven(simtime, pot, pid, controller, optimizer, simhours, dt=1.0,
              disturbances=None):
    disturbances = dict(disturbances or [])
    maxerrorever = 0.0
    maxerrorsinceengaged = 0.0
    everengaged = False
    steps = int(simhours * 3600 / dt)
    for i in range(steps):
        simtime["t"] = i * dt
        if i in disturbances:
            pot.temp += disturbances[i]
        pid.setpoint = controller.SP
        measured = round(pot.temp * 4) / 4.0  # MAX6675 quantization
        output = pid.compute(measured)
        controller.currentoutput = output
        pot.step(output)
        controller.currenttemp = round(pot.temp * 4) / 4.0
        optimizer.update(controller.currenttemp, setpoint=controller.SP, now=simtime["t"])
        error = abs(pot.temp - controller.SP)
        maxerrorever = max(maxerrorever, error)
        if optimizer.state not in ('waiting',):
            everengaged = True
        if everengaged:
            maxerrorsinceengaged = max(maxerrorsinceengaged, error)
    return maxerrorever, maxerrorsinceengaged


def test_never_exceeds_safety_limit_with_bad_starting_tunings():
    """Same real 'Turc' starting point that oscillated on the real
    fermenter (Kp=19.76) - prove the hard safety limit still holds."""
    badtunings = {"Kp": 19.75716534933873, "Ki": 0.029634692645012062, "Kd": 0.0}
    simtime, pot, pid, controller, optimizer = makerig(
        badtunings, maxswing=MAXSWING, windowseconds=15 * 60)

    maxerror, maxsinceengaged = rundriven(
        simtime, pot, pid, controller, optimizer, simhours=12)

    print("Max |temp - setpoint| over 12 simulated hours:", round(maxerror, 3),
          "C (since engaged:", round(maxsinceengaged, 3), "C)")
    trips = [e for e in optimizer.history if e.get('event') == 'safety_trip']
    windows = [e for e in optimizer.history if e.get('event') == 'window']
    print("Safety trips:", len(trips), "| Windows completed:", len(windows),
          "| Final state:", optimizer.state, "| Final tunings:", optimizer.currenttunings)

    assert maxsinceengaged <= MAXSWING + 1e-6, \
        "SAFETY VIOLATION: temperature strayed " + str(maxsinceengaged) + " C from setpoint (limit " + str(MAXSWING) + ")"
    assert windows, "optimizer should complete windows given the starting tunings settle within the limit here"
    print("OK - never exceeded the safety limit")


def test_kp_actually_decreases_and_search_terminates():
    """Core behaviour: starting from a comfortably-sufficient Kp/Ki on a
    warmed pot, Kp should be driven down over successive lowering_p windows
    (unlike the old gradient descent, this should be monotonic - never
    stepping back up during the lowering_p phase itself), and the whole
    search should reach a terminal 'holding' state within a bounded number
    of simulated hours rather than wandering forever."""
    starttunings = {"Kp": 20.0, "Ki": 0.03, "Kd": 0.0}
    simtime, pot, pid, controller, optimizer = makerig(
        starttunings, maxswing=MAXSWING, windowseconds=15 * 60, settleseconds=5 * 60)

    maxerror, maxsinceengaged = rundriven(
        simtime, pot, pid, controller, optimizer, simhours=16)

    kphistory = [e['tunings']['Kp'] for e in optimizer.history
                if e.get('event') == 'window' and e.get('phase') == 'lowering_p']
    print("Kp across lowering_p windows:", [round(k, 3) for k in kphistory])
    print("Final state:", optimizer.state, "final tunings:", optimizer.currenttunings)

    assert maxsinceengaged <= MAXSWING + 1e-6, "SAFETY VIOLATION: " + str(maxsinceengaged)
    assert len(kphistory) >= 2, "expected at least one successful step down in Kp"
    for a, b in zip(kphistory, kphistory[1:]):
        assert b <= a + 1e-9, "Kp must be monotonically non-increasing during lowering_p, got " + str(kphistory)
    assert optimizer.currenttunings['Kp'] < starttunings['Kp'], \
        "Kp should have been reduced from the starting point, got " + str(optimizer.currenttunings)
    assert optimizer.state in ('holding', 'raising_i'), \
        "expected the search to have reached raising_i or holding within 16h, got " + optimizer.state
    print("OK - Kp decreases monotonically and the search progresses toward a terminal state")


def test_cold_approach_with_extreme_starting_tunings_never_engages_unsafely():
    """Settle-gate must prevent the optimizer from ever trusting a cold
    approach as 'settled' before stored heat has dissipated - identical
    reasoning/property to PIDOptimizer's equivalent test."""
    huge = {"Kp": 200.0, "Ki": 0.5, "Kd": 0.0}
    simtime = {"t": 0.0}
    pot = TwoCapacityPot()
    pot.temp = 28.0
    pot.elemtemp = 28.0
    pid = SimulatedAdaptiveMCUPID()
    controller = FakeController(pid)
    controller.setSP(SETPOINT)
    controller.setaggPIDvalues(100.0, 0.0, 0.0)
    optimizer = MinPOptimizer(controller, huge, timesource=lambda: simtime["t"],
                              maxswing=MAXSWING, windowseconds=15 * 60)

    maxerror, maxsinceengaged = rundriven(
        simtime, pot, pid, controller, optimizer, simhours=10)

    print("Cold-approach huge-Kp start: max error (whole run)=", round(maxerror, 3),
          "since engaged=", round(maxsinceengaged, 3))
    assert maxsinceengaged <= MAXSWING + 1e-6, "SAFETY VIOLATION: " + str(maxsinceengaged)
    print("OK - settle-time gate prevents ever engaging unsafely during a cold approach")


def test_trips_and_recovers_from_a_mid_hold_disturbance():
    """A plausible-sized disturbance (-2 C, well under maxswing) mid-hold
    must trigger a safety trip if needed and the optimizer must recover
    and keep making progress afterward, not get stuck."""
    starttunings = {"Kp": 20.0, "Ki": 0.03, "Kd": 0.0}
    simtime, pot, pid, controller, optimizer = makerig(
        starttunings, maxswing=MAXSWING, windowseconds=15 * 60, settleseconds=60)

    maxerror, maxsinceengaged = rundriven(
        simtime, pot, pid, controller, optimizer, simhours=10,
        disturbances=[(600, -2.0)])

    trips = [e for e in optimizer.history if e.get('event') == 'safety_trip']
    windows = [e for e in optimizer.history if e.get('event') == 'window']
    print("Mid-hold disturbance: max error since engaged=", round(maxsinceengaged, 3),
          "trips=", len(trips), "windows=", len(windows))
    assert maxsinceengaged <= MAXSWING + 1e-6, "SAFETY VIOLATION: " + str(maxsinceengaged)
    assert windows, "optimizer never recovered enough to complete a window after the disturbance"
    print("OK - recovers from a mid-hold disturbance and keeps making progress")


def test_raises_ki_when_p_alone_cannot_hold():
    """Deterministic unit test of the phase-2 logic: force the optimizer
    straight into raising_i with a Kp that cannot possibly hold on its own
    (a plain, sustained heat-loss offset with no I term at all creates a
    permanent droop under P-only control), and verify Ki actually climbs
    until the mean error recovers under holdthreshold."""
    simtime, pot, pid, controller, optimizer = makerig(
        {"Kp": 5.0, "Ki": 0.0, "Kd": 0.0}, maxswing=MAXSWING,
        windowseconds=600, settleseconds=30, driftthreshold=0.3, holdthreshold=0.15)
    optimizer.state = 'lowering_p'
    optimizer.phaseaftercooldown = 'lowering_p'

    # Force straight into raising_i (skip the lowering_p search - this test
    # is about phase 2's own behaviour, not phase 1's).
    optimizer.state = 'raising_i'
    optimizer.windowstart = 0.0

    steps = int(6 * 3600)
    for i in range(steps):
        simtime["t"] = float(i)
        pid.setpoint = controller.SP
        measured = round(pot.temp * 4) / 4.0
        output = pid.compute(measured)
        pot.step(output)
        controller.currenttemp = round(pot.temp * 4) / 4.0
        optimizer.update(controller.currenttemp, setpoint=controller.SP, now=simtime["t"])

    kihistory = [e['tunings']['Ki'] for e in optimizer.history
                if e.get('event') == 'window' and e.get('phase') == 'raising_i']
    print("Ki across raising_i windows:", [round(k, 5) for k in kihistory])
    print("Final state:", optimizer.state, "tunings:", optimizer.currenttunings)
    assert kihistory, "expected at least one raising_i window"
    assert kihistory[-1] > 0.0, "Ki should have been raised from its 0.0 starting point"
    assert optimizer.state == 'holding', "expected the search to reach 'holding' within 6h, got " + optimizer.state
    print("OK - Ki is raised from 0 until holding is restored")


def test_stop_freezes_tunings():
    starttunings = {"Kp": 15.0, "Ki": 0.02, "Kd": 0.0}
    simtime, pot, pid, controller, optimizer = makerig(
        starttunings, maxswing=MAXSWING, windowseconds=600)

    rundriven(simtime, pot, pid, controller, optimizer, simhours=0.5)

    optimizer.stop()
    frozen = dict(optimizer.currenttunings)
    callsbefore = len(controller.pidcalls)
    rundriven(simtime, pot, pid, controller, optimizer, simhours=0.5)
    assert optimizer.currenttunings == frozen, "tunings changed after stop()"
    assert len(controller.pidcalls) == callsbefore, "controller was still being reconfigured after stop()"
    print("OK - stop() freezes tunings and stops updating")


def test_tripsafety_resumes_same_phase():
    """Deterministic unit test: a trip during raising_i must resume back
    into raising_i (not restart the whole search from lowering_p) once
    recovered."""
    simtime, pot, pid, controller, optimizer = makerig(
        {"Kp": 5.0, "Ki": 0.01, "Kd": 0.0}, maxswing=MAXSWING,
        windowseconds=900, settleseconds=30)
    optimizer.state = 'raising_i'
    optimizer.windowstart = 0.0
    optimizer.evercompletedwindow = True
    optimizer.safetunings = dict(optimizer.currenttunings)

    badtemp = SETPOINT + MAXSWING + 1.0
    optimizer.update(badtemp, setpoint=SETPOINT, now=0.0)

    assert optimizer.state == 'cooldown'
    assert optimizer.phaseaftercooldown == 'raising_i', \
        "must resume raising_i after cooldown, got " + optimizer.phaseaftercooldown
    assert any(m.startswith("SetSP(") for m in controller.sentmessages), \
        "expected a SetSP re-send to reset the firmware's accumulated integral term"

    cooldownlength = max(optimizer.windowseconds / 3.0, 120.0)
    for i in range(1, int(cooldownlength) + optimizer.settleseconds + 5):
        simtime["t"] = float(i)
        optimizer.update(SETPOINT + 0.1, setpoint=SETPOINT, now=simtime["t"])
    assert optimizer.state == 'raising_i', "should have resumed raising_i, got " + optimizer.state
    print("OK - a safety trip during raising_i resumes raising_i, not the whole search")


if __name__ == '__main__':
    test_never_exceeds_safety_limit_with_bad_starting_tunings()
    test_kp_actually_decreases_and_search_terminates()
    test_cold_approach_with_extreme_starting_tunings_never_engages_unsafely()
    test_trips_and_recovers_from_a_mid_hold_disturbance()
    test_raises_ki_when_p_alone_cannot_hold()
    test_stop_freezes_tunings()
    test_tripsafety_resumes_same_phase()
    print("\nOK - all MinPOptimizer tests passed.")
