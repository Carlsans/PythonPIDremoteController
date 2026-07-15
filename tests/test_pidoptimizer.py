"""Tests for PIDOptimizer: the online, gentle IAE-based tuner.

Uses TwoCapacityPot (fitted to the REAL pot's measured 38 C stored-heat
overshoot behaviour) and SimulatedAdaptiveMCUPID (firmware agg/cons
gap-switching), driven for many simulated hours. The single most important
property under test is the hard safety guarantee: the measured temperature
must NEVER exceed the configured maxswing from setpoint, even starting from
deliberately too-aggressive tunings (matching the real "Turc" profile that
caused persistent oscillation at 38 C in this session).

Run from the project root:
    ./venvarch/bin/python tests/test_pidoptimizer.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.PIDOptimizer import PIDOptimizer
from tests.simulator import TwoCapacityPot, SimulatedAdaptiveMCUPID, FakeController

SETPOINT = 38.0
MAXSWING = 3.0


def makewarmedpot():
    """A pot already holding near 38 C (established ferment, not a cold
    approach) - matches the real scenario the optimizer is meant for."""
    pot = TwoCapacityPot()
    pot.temp = SETPOINT
    pot.elemtemp = SETPOINT + 2.0  # a bit of residual stored heat, realistic
    return pot


def makerig(starttunings, **optimizerkwargs):
    """Builds pot/pid/controller/optimizer all sharing one simulated clock."""
    simtime = {"t": 0.0}
    pot = makewarmedpot()
    pid = SimulatedAdaptiveMCUPID()
    controller = FakeController(pid)
    controller.setSP(SETPOINT)
    controller.setaggPIDvalues(100.0, 0.0, 0.0)
    optimizer = PIDOptimizer(controller, starttunings, timesource=lambda: simtime["t"],
                             **optimizerkwargs)
    return simtime, pot, pid, controller, optimizer


def rundriven(simtime, pot, pid, controller, optimizer, simhours, dt=1.0,
              disturbances=None):
    """Returns (maxerrorever, maxerrorsinceengaged, iaehistory).

    maxerrorever includes any initial cold approach, where |temp-setpoint|
    legitimately exceeds maxswing before the optimizer ever engages (it
    stays in 'waiting' and does nothing until the temperature is already
    close - see PIDOptimizer.update()). maxerrorsinceengaged only counts
    from the moment it first leaves 'waiting', which is the property the
    safety guarantee actually applies to.

    disturbances: optional list of (simulated_second, delta_c) - e.g. adding
    cold ingredients mid-hold - applied directly to pot.temp at that time.
    """
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
        if optimizer.state != 'waiting':
            everengaged = True
        if everengaged:
            maxerrorsinceengaged = max(maxerrorsinceengaged, error)
    iaehistory = [e['iae'] for e in optimizer.history if e.get('event') == 'window']
    return maxerrorever, maxerrorsinceengaged, iaehistory


def test_never_exceeds_safety_limit_with_bad_starting_tunings():
    """The real 'Turc' profile (Kp=19.76) caused persistent oscillation at
    38 C on the real hardware tonight. Reproduce that starting point and
    prove the optimizer's hard safety limit actually holds - this is the
    property the user explicitly asked for ('should never exceed 3C')."""
    badtunings = {"Kp": 19.75716534933873, "Ki": 0.029634692645012062, "Kd": 0.0}
    simtime, pot, pid, controller, optimizer = makerig(
        badtunings, maxswing=MAXSWING, windowseconds=15 * 60, stepfraction=0.15)

    maxerror, maxsinceengaged, iaehistory = rundriven(
        simtime, pot, pid, controller, optimizer, simhours=10)

    print("Max |temp - setpoint| over 10 simulated hours:", round(maxerror, 3),
          "C (since engaged:", round(maxsinceengaged, 3), "C)")
    trips = [e for e in optimizer.history if e.get('event') == 'safety_trip']
    windows = [e for e in optimizer.history if e.get('event') == 'window']
    print("Safety trips:", len(trips), "| Windows completed:", len(windows))
    print("Final tunings:", optimizer.currenttunings)

    assert maxsinceengaged <= MAXSWING + 1e-6, \
        "SAFETY VIOLATION: temperature strayed " + str(maxsinceengaged) + " C from setpoint (limit " + str(MAXSWING) + ")"
    assert windows, "optimizer should complete windows given the starting tunings settle within the limit here"
    print("OK - never exceeded the safety limit")


def test_tripsafety_mechanism_directly():
    """Deterministic unit test of the trip logic itself: settle in, build up
    partial-window state, then feed one update with |temp-setpoint| >
    maxswing, and verify the mechanism reacts correctly - this does not
    depend on any thermal model happening to produce a large excursion, so
    it is the most reliable proof the safety net works. Uses a short
    settleseconds/windowseconds so the test stays fast and its timings are
    easy to reason about explicitly."""
    simtime, pot, pid, controller, optimizer = makerig(
        {"Kp": 19.76, "Ki": 0.03, "Kd": 0.0}, maxswing=MAXSWING,
        windowseconds=900, settleseconds=30)

    # First 30s: settling. Must not yet be 'running'.
    for i in range(30):
        simtime["t"] = float(i)
        optimizer.update(SETPOINT + 0.1, setpoint=SETPOINT, now=simtime["t"])
    assert optimizer.state == 'waiting', "must still be settling, got " + optimizer.state

    # A few more seconds past the settle threshold: now engaged and running.
    for i in range(30, 40):
        simtime["t"] = float(i)
        optimizer.update(SETPOINT + 0.1, setpoint=SETPOINT, now=simtime["t"])
    assert optimizer.state == 'running'
    assert not optimizer.history, "no trip or window should have happened yet"

    simtime["t"] = 40.0
    badtemp = SETPOINT + MAXSWING + 1.0  # unambiguously over the hard limit
    optimizer.update(badtemp, setpoint=SETPOINT, now=simtime["t"])

    assert optimizer.state == 'cooldown', "must enter cooldown immediately on a limit breach"
    assert optimizer.history and optimizer.history[-1]['event'] == 'safety_trip'
    assert optimizer.currenttunings['Kp'] < 19.76, \
        "tunings must be backed off (reduced), got " + str(optimizer.currenttunings)

    # Re-sending the setpoint to force an integral reset must have happened.
    assert any(m.startswith("SetSP(") for m in controller.sentmessages), \
        "expected a SetSP re-send to reset the firmware's accumulated integral term"

    # During cooldown, further updates (even safe ones) must NOT change
    # tunings again until the cooldown period AND the settle period elapse.
    frozen = dict(optimizer.currenttunings)
    cooldownlength = max(optimizer.windowseconds / 3.0, 120.0)
    for i in range(41, 41 + int(cooldownlength) + optimizer.settleseconds + 5):
        simtime["t"] = float(i)
        optimizer.update(SETPOINT + 0.1, setpoint=SETPOINT, now=simtime["t"])
    assert optimizer.state == 'running', "should have recovered and re-engaged by now"
    assert optimizer.currenttunings == frozen, \
        "tunings must not change again until fully recovered and a new window completes"

    print("OK - tripsafety() mechanism reacts correctly, resets the integral, "
          "and freezes tunings until fully recovered")


def test_cold_approach_with_extreme_starting_tunings_never_engages_unsafely():
    """A cold approach with deliberately extreme starting tunings (Kp=200):
    the settle-time gate (see PIDOptimizer.settleseconds) must let the
    stored heat from the aggressive-profile climb dissipate BEFORE ever
    trusting the temperature as 'settled', so the optimizer never even
    engages during the dangerous window - prevention rather than reacting
    after the fact, which earlier iterations of this test proved a
    reactive-only trip cannot always contain (physical stored heat can
    keep arriving after any software reaction, regardless of how quickly
    tunings are reverted)."""
    huge = {"Kp": 200.0, "Ki": 0.5, "Kd": 0.0}
    simtime = {"t": 0.0}
    pot = TwoCapacityPot()
    pot.temp = 28.0
    pot.elemtemp = 28.0
    pid = SimulatedAdaptiveMCUPID()
    controller = FakeController(pid)
    controller.setSP(SETPOINT)
    controller.setaggPIDvalues(100.0, 0.0, 0.0)
    optimizer = PIDOptimizer(controller, huge, timesource=lambda: simtime["t"],
                             maxswing=MAXSWING, windowseconds=15 * 60, stepfraction=0.15)

    maxerror, maxsinceengaged, iaehistory = rundriven(
        simtime, pot, pid, controller, optimizer, simhours=10)

    print("Cold-approach huge-Kp start: max error (whole run)=", round(maxerror, 3),
          "since engaged=", round(maxsinceengaged, 3))
    # maxerror itself is expected to be large here (a fresh approach from
    # 28 C legitimately passes through a big error on the way to 38 C -
    # the optimizer correctly ignores that, staying in 'waiting'). The
    # guarantee only applies once it has actually engaged.
    assert maxsinceengaged <= MAXSWING + 1e-6, "SAFETY VIOLATION: " + str(maxsinceengaged)
    print("OK - settle-time gate prevents ever engaging unsafely during a cold approach")


def test_trips_and_recovers_from_a_mid_hold_disturbance():
    """Once already engaged and settled, a sudden disturbance (e.g. opening
    the lid, a small amount of cooler ingredient) must trigger a safety
    trip if the currently-active (aggressive) tunings would otherwise let
    the temperature swing too far while recovering, and the optimizer must
    actually recover afterward instead of getting stuck.

    The disturbance itself (2 C) is deliberately kept under maxswing: an
    INSTANTANEOUS external shock bigger than maxswing (e.g. dumping in a
    liter of cold milk) cannot possibly be kept under maxswing by any
    controller - the guarantee this class provides is about what its OWN
    tuning changes cause, and about reacting to and recovering from a
    disturbance without making it worse, not about defying physics for an
    arbitrarily large external shock.
    """
    huge = {"Kp": 60.0, "Ki": 0.15, "Kd": 0.0}
    simtime, pot, pid, controller, optimizer = makerig(
        huge, maxswing=MAXSWING, windowseconds=15 * 60, stepfraction=0.15, settleseconds=60)

    # -2 C, ten simulated minutes in, well after the optimizer has settled
    # and engaged (settleseconds=60s here).
    maxerror, maxsinceengaged, iaehistory = rundriven(
        simtime, pot, pid, controller, optimizer, simhours=10,
        disturbances=[(600, -2.0)])

    trips = [e for e in optimizer.history if e.get('event') == 'safety_trip']
    windows = [e for e in optimizer.history if e.get('event') == 'window']
    print("Mid-hold disturbance: max error since engaged=", round(maxsinceengaged, 3),
          "trips=", len(trips), "windows=", len(windows))
    assert maxsinceengaged <= MAXSWING + 1e-6, "SAFETY VIOLATION: " + str(maxsinceengaged)
    assert trips, "expected the disturbance to trigger at least one safety trip"
    assert windows, "optimizer never recovered enough to complete a window after tripping"
    print("OK - trips on a mid-hold disturbance and recovers")


def test_iae_improves_over_time():
    """Starting from tunings that oscillate a lot (but not enough to trip
    safety), IAE across successive windows should trend down as the
    optimizer nudges parameters - i.e. it actually improves control, not
    just avoids catastrophe."""
    starttunings = {"Kp": 12.0, "Ki": 0.02, "Kd": 0.0}
    simtime, pot, pid, controller, optimizer = makerig(
        starttunings, maxswing=MAXSWING, windowseconds=15 * 60, stepfraction=0.15)

    maxerror, maxsinceengaged, iaehistory = rundriven(
        simtime, pot, pid, controller, optimizer, simhours=14)

    assert maxsinceengaged <= MAXSWING + 1e-6, "safety limit violated: " + str(maxsinceengaged)
    assert len(iaehistory) >= 6, "not enough windows completed to judge improvement: " + str(len(iaehistory))
    firstthird = iaehistory[:len(iaehistory) // 3]
    lastthird = iaehistory[-len(iaehistory) // 3:]
    meanfirst = sum(firstthird) / len(firstthird)
    meanlast = sum(lastthird) / len(lastthird)
    print("Mean IAE first third:", round(meanfirst, 2), "-> last third:", round(meanlast, 2),
          "(", len(iaehistory), "windows)")
    assert meanlast < meanfirst, \
        "IAE did not improve: first third mean=" + str(meanfirst) + " last third mean=" + str(meanlast)
    print("OK - IAE improves over successive windows")


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


if __name__ == '__main__':
    test_never_exceeds_safety_limit_with_bad_starting_tunings()
    test_tripsafety_mechanism_directly()
    test_cold_approach_with_extreme_starting_tunings_never_engages_unsafely()
    test_trips_and_recovers_from_a_mid_hold_disturbance()
    test_iae_improves_over_time()
    test_stop_freezes_tunings()
    print("\nOK - all PIDOptimizer tests passed.")
