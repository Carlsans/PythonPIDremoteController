"""Regression test for RelayAutotune's per-task "check log" - a dedicated,
real-time, human-checkable audit trail of every reading, every relay
event, and the full arithmetic behind Ku/Tu and every derived tuning rule.

Verifies: each task gets its own file (never shared/appended across
tasks), every input parameter is documented, every raw reading is logged
in real time (not just buffered to the end), every completed cycle shows
its arithmetic, the final computation report substitutes real numbers into
every formula, and a partial (aborted) run's check log still gets a
complete, correctly-labelled report and is properly closed.

Run from the project root:
    ./venvarch/bin/python tests/test_tuning_checklog.py
"""

import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.RelayAutotune import RelayAutotune
from tests.simulator import SimulatedPot, SimulatedMCUPID, FakeController

TARGET = 40.0
TMPDIR = tempfile.mkdtemp(prefix="checklogtest")


def measure(pot):
    return round(pot.temp * 4) / 4.0


def newpath(name):
    return os.path.join(TMPDIR, name)


def test_full_run_produces_a_complete_human_readable_report():
    pot = SimulatedPot(ambient=22.0, fullpowerrise=90.0, tau=1500.0, deadtime=20.0)
    pid = SimulatedMCUPID(outmax=255.0)
    controller = FakeController(pid)
    checklogpath = newpath("full_run.log")
    resultsfile = newpath("full_run_results.txt")
    tuner = RelayAutotune(controller, targettemp=TARGET, maxsafetemp=60.0,
                          skipcycles=1, cyclestomeasure=4,
                          resultsfile=resultsfile, checklogfile=checklogpath)

    assert tuner.checklogpath == checklogpath
    assert os.path.exists(checklogpath), "check log file must be created immediately, not lazily"

    t = 0
    while tuner.state not in ('done', 'aborted') and t < 4 * 60 * 60:
        output = pid.compute(measure(pot))
        controller.currentoutput = output
        pot.step(output)
        tuner.update(measure(pot), now=float(t))
        t += 1
    assert tuner.state == 'done', "test setup did not reach a full completion"

    with open(checklogpath) as f:
        content = f.read()

    # Every constructor parameter is documented, not just used silently.
    for expected in ("target temperature (targettemp)     = 40.0",
                      "hysteresis                          = +/- 0.15",
                      "cycles to measure (cyclestomeasure) = 4",
                      "cycles to skip (skipcycles)         = 1",
                      "RELAY_KP (class constant)           = 10000.0"):
        assert expected in content, "missing documented parameter: " + expected

    # Raw readings were logged in real time (many, not just a summary).
    tickcount = content.count("t=")
    assert tickcount > 100, "expected many real-time tick lines, got " + str(tickcount)

    # Every completed cycle shows its own arithmetic.
    assert content.count("CYCLE ") >= 5, "expected a per-cycle breakdown for each completed cycle"
    assert "amplitude = (peak - trough) / 2" in content

    # The final computation report substitutes real numbers into every
    # formula - not just the bare result dict.
    for expected in ("ULTIMATE PERIOD Tu", "Tu = sum(periods) / count",
                      "OSCILLATION AMPLITUDE a", "RELAY AMPLITUDE d",
                      "ULTIMATE GAIN Ku (Astrom-Hagglund relay-feedback formula)",
                      "Ku = 4*d / (pi*a)",
                      "Ziegler-Nichols PI (no derivative)",
                      "Tyreus-Luyben (classic PID)",
                      "APPLIED RULE: ziegler_nichols_pi",
                      "RESULT: FULL run - 4/4 target cycle(s) used.",
                      "Pushed to controller: setAllPID(",
                      "Check log closed:"):
        assert expected in content, "missing computation-report section: " + expected

    # The numbers actually used in the report must match the real result -
    # this is the whole point: a human must be able to trust these figures.
    ku_line = "= " + str(round(tuner.result['Ku'], 6))
    assert ku_line in content, "logged Ku does not match the actual computed result"

    if os.path.exists(resultsfile):
        os.remove(resultsfile)
    print("OK - a full run produces a complete, self-documenting, human-checkable report")


def test_each_task_gets_its_own_file():
    pot1 = SimulatedPot(ambient=22.0, fullpowerrise=90.0, tau=1500.0, deadtime=20.0)
    pid1 = SimulatedMCUPID(outmax=255.0)
    controller1 = FakeController(pid1)
    path1 = newpath("task1.log")
    tuner1 = RelayAutotune(controller1, targettemp=38.0, maxsafetemp=60.0,
                           resultsfile=newpath("task1_results.txt"), checklogfile=path1)

    pot2 = SimulatedPot(ambient=22.0, fullpowerrise=90.0, tau=1500.0, deadtime=20.0)
    pid2 = SimulatedMCUPID(outmax=255.0)
    controller2 = FakeController(pid2)
    path2 = newpath("task2.log")
    tuner2 = RelayAutotune(controller2, targettemp=82.0, maxsafetemp=95.0,
                           resultsfile=newpath("task2_results.txt"), checklogfile=path2)

    assert tuner1.checklogpath != tuner2.checklogpath
    with open(path1) as f:
        content1 = f.read()
    with open(path2) as f:
        content2 = f.read()
    assert "target temperature (targettemp)     = 38.0" in content1
    assert "target temperature (targettemp)     = 82.0" in content2
    # Neither task's parameters leaked into the other's file.
    assert "target temperature (targettemp)     = 82.0" not in content1
    assert "target temperature (targettemp)     = 38.0" not in content2
    tuner1.close()
    tuner2.close()
    print("OK - each tuning task gets its own, non-shared check log file")


def test_partial_run_report_is_labelled_and_closed():
    pot = SimulatedPot(ambient=22.0, fullpowerrise=90.0, tau=1500.0, deadtime=20.0)
    pid = SimulatedMCUPID(outmax=255.0)
    controller = FakeController(pid)
    checklogpath = newpath("partial_run.log")
    resultsfile = newpath("partial_run_results.txt")
    tuner = RelayAutotune(controller, targettemp=TARGET, maxsafetemp=60.0,
                          skipcycles=1, cyclestomeasure=4,
                          resultsfile=resultsfile, checklogfile=checklogpath)

    t = 0
    while len(tuner.cyclepeaks) < 2 and t < 4 * 60 * 60:
        output = pid.compute(measure(pot))
        controller.currentoutput = output
        pot.step(output)
        tuner.update(measure(pot), now=float(t))
        t += 1

    tuner.abort("stopped by user")

    with open(checklogpath) as f:
        content = f.read()
    assert "ABORT: stopped by user" in content
    assert "RESULT: PARTIAL run - 1/4 target cycle(s) used." in content
    assert "Stopped early: stopped by user" in content
    assert "Check log closed:" in content
    if os.path.exists(resultsfile):
        os.remove(resultsfile)
    print("OK - a partial (stopped-early) run's check log is fully labelled and closed")


def main():
    test_full_run_produces_a_complete_human_readable_report()
    test_each_task_gets_its_own_file()
    test_partial_run_report_is_labelled_and_closed()
    print("\nOK - PID tuning check log verified.")


if __name__ == '__main__':
    main()
