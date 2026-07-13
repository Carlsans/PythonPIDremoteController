"""Automated test of the relay autotuner against a simulated pot.

Runs entirely in simulated time (instant). The sensor is quantized to
0.25 C steps like the real MAX6675. Verifies that:
1. the autotune converges and produces sane Ku/Tu values,
2. the applied tunings control the simulated pot from a cold start with
   small overshoot and no steady-state error,
3. the loop recovers from a disturbance (e.g. adding cold starter).

Run from the project root:
    ./venvarch/bin/python tests/test_relayautotune.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.RelayAutotune import RelayAutotune
from tests.simulator import SimulatedPot, SimulatedMCUPID, FakeController

TARGET = 40.0


def measure(pot):
    """MAX6675 quantization: 0.25 C steps."""
    return round(pot.temp * 4) / 4.0


def run_autotune(outmax=255.0):
    pot = SimulatedPot(ambient=22.0, fullpowerrise=90.0, tau=1500.0, deadtime=20.0)
    pid = SimulatedMCUPID(outmax=outmax)
    controller = FakeController(pid)
    resultsfile = os.path.join(os.path.dirname(os.path.abspath(__file__)), "autotune_results_test.txt")
    if os.path.exists(resultsfile):
        os.remove(resultsfile)
    tuner = RelayAutotune(controller, targettemp=TARGET, maxsafetemp=60.0,
                          resultsfile=resultsfile)

    maxsimseconds = 4 * 60 * 60
    for t in range(maxsimseconds):
        output = pid.compute(measure(pot))
        controller.currentoutput = output  # like the real "Output:" messages
        pot.step(output)
        tuner.update(measure(pot), now=float(t))
        if tuner.state in ('done', 'aborted'):
            break

    assert tuner.state == 'done', "autotune did not finish, state=" + tuner.state
    assert os.path.exists(resultsfile), "results file was not written"
    os.remove(resultsfile)
    return pot, pid, controller, tuner, t


def run_closedloop(pot, pid, seconds, disturbat=None, disturbdelta=0.0):
    """Run the loop; return (overshoot after first crossing, temps of last 30 min)."""
    overshoot = 0.0
    crossed = False
    finaltemps = []
    for t in range(seconds):
        if disturbat is not None and t == disturbat:
            pot.temp += disturbdelta
        output = pid.compute(measure(pot))
        pot.step(output)
        if pot.temp >= TARGET:
            crossed = True
        if crossed and (disturbat is None or t < disturbat):
            overshoot = max(overshoot, pot.temp - TARGET)
        if t >= seconds - 30 * 60:
            finaltemps.append(pot.temp)
    return crossed, overshoot, finaltemps


def main():
    pot, pid, controller, tuner, tunetime = run_autotune()
    result = tuner.result
    print("\nAutotune finished after", tunetime, "simulated seconds (",
          round(tunetime / 60.0, 1), "min )")

    assert result['Ku'] > 0, "Ku must be positive"
    assert 30 < result['Tu'] < 3600, "Tu implausible: " + str(result['Tu'])
    applied = controller.pidcalls[-1]
    znpi = result['ziegler_nichols_pi']
    assert result['applied'] == 'ziegler_nichols_pi'
    assert abs(applied[0] - znpi['Kp']) < 1e-9, "Ziegler-Nichols PI Kp was not applied"
    assert controller.SP == TARGET, "setpoint was not restored to the target"
    assert tunetime < 2 * 60 * 60, "autotune took more than 2 simulated hours"

    # Cold-start verification with the applied tunings.
    pot2 = SimulatedPot(ambient=22.0, fullpowerrise=90.0, tau=1500.0, deadtime=20.0)
    pid.iterm = 0.0
    pid.lastinput = None
    crossed, overshoot, finaltemps = run_closedloop(pot2, pid, 3 * 60 * 60)
    finalerror = max(abs(temp - TARGET) for temp in finaltemps)
    print("Cold start: overshoot =", round(overshoot, 3),
          "C, max |error| in last 30 min =", round(finalerror, 4), "C")
    assert crossed, "temperature never reached the target"
    # ~2 C of cold-start overshoot comes from integral windup in the MCU's
    # PID during the full-power climb, not from the tunings; in real use the
    # hold temperature is approached from above after pasteurizing.
    assert overshoot < 2.5, "overshoot too large: " + str(overshoot)
    assert finalerror < 0.4, "does not settle at target, error=" + str(finalerror)

    # Disturbance rejection: dump 5 C of cold starter into the pot.
    crossed, _, finaltemps = run_closedloop(pot2, pid, 90 * 60,
                                            disturbat=60, disturbdelta=-5.0)
    finalerror = max(abs(temp - TARGET) for temp in finaltemps)
    print("Disturbance: max |error| 30 min after -5C disturbance =",
          round(finalerror, 4), "C")
    assert finalerror < 0.4, "does not recover from disturbance, error=" + str(finalerror)

    # The real firmware clamps its output at 160/255: the tuner must use the
    # observed swing as the relay amplitude, not the nominal 255 span.
    _, _, _, tunerclamped, _ = run_autotune(outmax=160.0)
    d = tunerclamped.result['relay_amplitude']
    assert abs(d - 80.0) < 2.0, "relay amplitude should be ~80 for a 160 clamp, got " + str(d)
    # Ku is a property of the pot, not of the relay: measuring the actual
    # swing must give (almost) the same Ku as the unclamped run. With the old
    # hardcoded d=127.5 this ratio was ~1.6, i.e. dangerously hot tunings.
    ratio = tunerclamped.result['Ku'] / result['Ku']
    assert 0.8 < ratio < 1.25, "clamped-output Ku should match the process Ku (got ratio " + str(round(ratio, 2)) + ")"

    print("\nOK - relay autotune converges and its tunings control the pot.")


if __name__ == '__main__':
    main()
