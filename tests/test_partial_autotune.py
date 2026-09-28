"""Regression test for RelayAutotune's partial-results-on-abort behaviour.

Relay autotune is slow (tens of minutes per cycle on real hardware), so a
user stopping it early - or a timeout, or a safety abort - used to discard
everything and call oncomplete(None). This verifies that whatever complete
cycles were already measured (even just one) are still turned into tunings
and reported, and that a genuinely empty run still reports None cleanly.

Run from the project root:
    ./venvarch/bin/python tests/test_partial_autotune.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.RelayAutotune import RelayAutotune
from tests.simulator import SimulatedPot, SimulatedMCUPID, FakeController

TARGET = 40.0


def measure(pot):
    return round(pot.temp * 4) / 4.0


def newresultsfile(name):
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), name)
    if os.path.exists(path):
        os.remove(path)
    return path


def test_stop_after_one_cycle_yields_partial_result():
    pot = SimulatedPot(ambient=22.0, fullpowerrise=90.0, tau=1500.0, deadtime=20.0)
    pid = SimulatedMCUPID(outmax=255.0)
    controller = FakeController(pid)
    resultsfile = newresultsfile("autotune_results_partial_test.txt")
    completions = []
    tuner = RelayAutotune(controller, targettemp=TARGET, maxsafetemp=60.0,
                          skipcycles=1, cyclestomeasure=4,
                          resultsfile=resultsfile, oncomplete=completions.append)

    t = 0
    # Run until exactly 2 cycles have completed (1 skip + 1 measured) - well
    # short of the 5 (1 skip + 4 measure) a full run needs.
    while len(tuner.cyclepeaks) < 2 and t < 4 * 60 * 60:
        output = pid.compute(measure(pot))
        controller.currentoutput = output
        pot.step(output)
        tuner.update(measure(pot), now=float(t))
        t += 1
    assert len(tuner.cyclepeaks) == 2, "test setup did not reach 2 cycles"
    assert tuner.state == 'relay', "should not have finished naturally yet"

    tuner.abort("stopped by user")

    assert tuner.state == 'aborted'
    assert tuner.result is not None, "partial data was available but no result was computed"
    assert tuner.result['cycles_used'] == 1, "expected exactly 1 post-skip cycle"
    assert tuner.result['cycles_target'] == 4
    assert tuner.result['cycles_used'] < tuner.result['cycles_target']
    assert tuner.result['aborted_reason'] == "stopped by user"
    assert tuner.result['Ku'] > 0 and tuner.result['Tu'] > 0
    assert len(completions) == 1 and completions[0] is tuner.result, \
        "oncomplete must fire exactly once, with the partial result"
    assert os.path.exists(resultsfile), "partial result was not persisted to the results file"
    os.remove(resultsfile)

    # setAllPID/setSP must NOT have been called with the partial tunings -
    # unlike a full completion, a partial result is reported, not applied.
    assert controller.pidcalls == [(RelayAutotune.RELAY_KP, 0.0, 0.0)], \
        "partial tunings should not be pushed to the controller automatically"
    print("OK - stopping after 1 measured cycle still yields a usable partial result:",
          "Ku=" + str(round(tuner.result['Ku'], 2)), "Tu=" + str(round(tuner.result['Tu'], 1)))


def test_abort_with_zero_cycles_reports_none():
    pot = SimulatedPot(ambient=22.0, fullpowerrise=90.0, tau=1500.0, deadtime=20.0)
    pid = SimulatedMCUPID(outmax=255.0)
    controller = FakeController(pid)
    resultsfile = newresultsfile("autotune_results_empty_test.txt")
    completions = []
    tuner = RelayAutotune(controller, targettemp=TARGET, maxsafetemp=60.0,
                          resultsfile=resultsfile, oncomplete=completions.append)

    # A couple of updates, not even one full cycle yet.
    for t in range(5):
        output = pid.compute(measure(pot))
        controller.currentoutput = output
        pot.step(output)
        tuner.update(measure(pot), now=float(t))

    tuner.abort("stopped by user")

    assert tuner.state == 'aborted'
    assert tuner.result is None, "there was no oscillation data - nothing should have been computed"
    assert completions == [None], "oncomplete must fire exactly once, with None"
    assert not os.path.exists(resultsfile), "nothing should have been written with zero usable cycles"
    print("OK - aborting with zero measured cycles reports None cleanly")


def test_abort_is_idempotent():
    pot = SimulatedPot(ambient=22.0, fullpowerrise=90.0, tau=1500.0, deadtime=20.0)
    pid = SimulatedMCUPID(outmax=255.0)
    controller = FakeController(pid)
    resultsfile = newresultsfile("autotune_results_idempotent_test.txt")
    completions = []
    tuner = RelayAutotune(controller, targettemp=TARGET, maxsafetemp=60.0,
                          skipcycles=1, cyclestomeasure=4,
                          resultsfile=resultsfile, oncomplete=completions.append)

    t = 0
    while len(tuner.cyclepeaks) < 2 and t < 4 * 60 * 60:
        output = pid.compute(measure(pot))
        controller.currentoutput = output
        pot.step(output)
        tuner.update(measure(pot), now=float(t))
        t += 1

    tuner.abort("first stop")
    firstresult = tuner.result
    tuner.abort("second stop (should be a no-op)")

    assert tuner.result is firstresult, "a second abort() must not recompute or replace the result"
    assert len(completions) == 1, "oncomplete must not fire again on a second abort()"
    if os.path.exists(resultsfile):
        os.remove(resultsfile)
    print("OK - abort() on an already-aborted tuner is a no-op")


def main():
    test_stop_after_one_cycle_yields_partial_result()
    test_abort_with_zero_cycles_reports_none()
    test_abort_is_idempotent()
    print("\nOK - partial-results-on-abort behaviour verified.")


if __name__ == '__main__':
    main()
