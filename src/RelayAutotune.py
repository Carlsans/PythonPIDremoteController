import datetime
import time


class RelayAutotune:
    """Relay-feedback (Astrom-Hagglund) PID autotuner.

    Instead of gradient descent (which needs a ~40 min experiment per
    measurement), this makes the pot oscillate around the target temperature
    and derives the PID gains from the oscillation in a handful of cycles.

    How the relay is built with the existing MCU commands: the MCU tunings
    are set to a huge pure-P gain, so its PID output saturates to 255 when
    the temperature is below the setpoint and to 0 when above - i.e. it acts
    as an on/off relay. This program then toggles the setpoint between
    "well above target" (heater fully on) and "well below target" (heater
    off) whenever the measured temperature crosses the target, with a small
    hysteresis so sensor noise cannot cause chatter.

    From the sustained oscillation we measure the amplitude `a` and the
    period `Tu`, compute the ultimate gain Ku = 4*d / (pi*a) (d = half the
    output span) and derive tunings with Ziegler-Nichols and Tyreus-Luyben
    rules. Tyreus-Luyben is applied by default: it is more conservative and
    behaves better on slow, lag-dominated thermal processes like a pot of
    milk, where overshoot must stay small.

    Safety: the "heater on" setpoint sent to the MCU is bounded, so even if
    this program dies mid-tune the pot cannot run away past maxsafetemp.
    """

    OUTPUT_SPAN = 255.0  # MCU PID output range
    RELAY_KP = 10000.0   # pure-P gain that saturates the MCU PID either way

    def __init__(self, controller, targettemp=40.0, hysteresis=0.15,
                 cyclestomeasure=4, skipcycles=2, maxsafetemp=60.0,
                 timeoutseconds=4 * 60 * 60, timesource=time.time,
                 resultsfile="autotune_results.txt", oncomplete=None):
        self.controller = controller
        self.targettemp = targettemp
        self.hysteresis = hysteresis
        self.cyclestomeasure = cyclestomeasure
        self.skipcycles = skipcycles
        self.maxsafetemp = maxsafetemp
        self.timeoutseconds = timeoutseconds
        self.now = timesource
        self.resultsfile = resultsfile
        # Called with the result dict when tuning finishes, or None on abort.
        self.oncomplete = oncomplete

        self.state = 'init'  # init -> relay -> done | aborted
        self.relayon = None
        self.starttime = None
        self.switchtimes = []  # times of off->on relay edges (cycle starts)
        self.cyclepeaks = []   # max temp seen in each completed cycle
        self.cycletroughs = []  # min temp seen in each completed cycle
        self.currentmax = None
        self.currentmin = None
        # Observed raw MCU output extremes. The firmware may clamp its output
        # below the nominal 255 (the real device clamps at 160), so the relay
        # amplitude must come from the actual swing, not the nominal span.
        self.observedmaxout = None
        self.observedminout = None
        self.result = None

    # ------------------------------------------------------------------
    # Relay actuation
    # ------------------------------------------------------------------
    def setrelaytunings(self):
        self.controller.setAllPID(self.RELAY_KP, 0.0, 0.0)

    def relay(self, on):
        if on == self.relayon:
            return
        self.relayon = on
        if on:
            # Setpoint above target saturates the huge-Kp PID to full output,
            # but stays bounded so a dead controller cannot overheat the pot.
            self.controller.setSP(min(self.targettemp + 15, self.maxsafetemp))
        else:
            # Setpoint far below target -> output 0.
            self.controller.setSP(1)

    # ------------------------------------------------------------------
    # Main entry point: call once per second with the measured temperature
    # ------------------------------------------------------------------
    def update(self, temp, now=None):
        if self.state in ('done', 'aborted'):
            return
        if now is None:
            now = self.now()
        if temp is None or temp <= 0:
            return  # no valid sensor reading yet

        if temp > self.maxsafetemp:
            self.controller.setSP(1)
            self.abort("temperature " + str(temp) + " above safe maximum " + str(self.maxsafetemp))
            return

        if self.state == 'init':
            self.starttime = now
            self.setrelaytunings()
            self.relay(temp < self.targettemp)
            self.state = 'relay'
            print("Relay autotune started. Target =", self.targettemp,
                  "hysteresis = +/-", self.hysteresis)
            return

        if now - self.starttime > self.timeoutseconds:
            self.controller.setSP(1)
            self.abort("timeout after " + str(self.timeoutseconds) + "s without enough oscillations")
            return

        output = getattr(self.controller, 'currentoutput', None)
        if output is not None:
            if self.observedmaxout is None or output > self.observedmaxout:
                self.observedmaxout = output
            if self.observedminout is None or output < self.observedminout:
                self.observedminout = output

        # Track the extremes of the current cycle.
        if self.currentmax is None or temp > self.currentmax:
            self.currentmax = temp
        if self.currentmin is None or temp < self.currentmin:
            self.currentmin = temp

        if self.relayon and temp >= self.targettemp + self.hysteresis:
            self.relay(False)
        elif not self.relayon and temp <= self.targettemp - self.hysteresis:
            self.oncyclestart(now)
            if self.state == 'relay':  # oncyclestart may have finished the tune
                self.relay(True)

    def oncyclestart(self, now):
        """An off->on edge: one full oscillation cycle just completed."""
        if self.switchtimes:
            self.cyclepeaks.append(self.currentmax)
            self.cycletroughs.append(self.currentmin)
            period = now - self.switchtimes[-1]
            print("Autotune cycle " + str(len(self.cyclepeaks)) + " done: period=" + str(round(period, 1))
                  + "s peak=" + str(self.currentmax) + " trough=" + str(self.currentmin))
        self.switchtimes.append(now)
        self.currentmax = None
        self.currentmin = None
        if len(self.cyclepeaks) >= self.skipcycles + self.cyclestomeasure:
            self.computetunings()

    # ------------------------------------------------------------------
    # Tuning computation
    # ------------------------------------------------------------------
    def computetunings(self):
        peaks = self.cyclepeaks[self.skipcycles:]
        troughs = self.cycletroughs[self.skipcycles:]
        periods = []
        for i in range(len(self.switchtimes) - 1):
            periods.append(self.switchtimes[i + 1] - self.switchtimes[i])
        periods = periods[self.skipcycles:]

        Tu = sum(periods) / len(periods)
        amplitudes = [(p - t) / 2.0 for p, t in zip(peaks, troughs)]
        a = sum(amplitudes) / len(amplitudes)
        if a <= 0 or Tu <= 0:
            self.abort("degenerate oscillation (amplitude=" + str(a) + ", period=" + str(Tu) + ")")
            return
        if (self.observedmaxout is not None and self.observedminout is not None
                and self.observedmaxout - self.observedminout > 1.0):
            d = (self.observedmaxout - self.observedminout) / 2.0
        else:
            d = self.OUTPUT_SPAN / 2.0
        pi = 3.141592653589793
        Ku = 4.0 * d / (pi * a)

        zn = {'Kp': 0.6 * Ku, 'Ki': 1.2 * Ku / Tu, 'Kd': 0.075 * Ku * Tu}
        tl = {'Kp': 0.454 * Ku, 'Ki': 0.454 * Ku / (2.2 * Tu), 'Kd': 0.454 * Ku * Tu / 6.3}
        # PI variants (no derivative: the MAX6675 reads in 0.25 C steps and a
        # large Kd kicks the output hard on every quantization step).
        # ZN-PI is applied: on the real pot the TL-PI integral proved too slow
        # to supply the steady power a hot setpoint needs (8.7 C droop during
        # a 10 min hold at 82 C), and ZN-PI reproduces the proven hand tuning.
        znpi = {'Kp': 0.45 * Ku, 'Ki': 0.54 * Ku / Tu, 'Kd': 0.0}
        tlpi = {'Kp': Ku / 3.2, 'Ki': Ku / (3.2 * 2.2 * Tu), 'Kd': 0.0}
        noovershoot = {'Kp': 0.2 * Ku, 'Ki': 0.4 * Ku / Tu, 'Kd': 0.0667 * Ku * Tu}

        self.result = {
            'Ku': Ku, 'Tu': Tu, 'amplitude': a, 'relay_amplitude': d,
            'ziegler_nichols': zn,
            'ziegler_nichols_pi': znpi,
            'tyreus_luyben': tl,
            'tyreus_luyben_pi': tlpi,
            'no_overshoot': noovershoot,
            'applied': 'ziegler_nichols_pi',
        }
        print("Relay autotune finished.")
        print("  Ultimate gain Ku =", Ku, " ultimate period Tu =", Tu, "s, amplitude =", a)
        print("  Ziegler-Nichols     :", zn)
        print("  Ziegler-Nichols PI  :", znpi, "(applied)")
        print("  Tyreus-Luyben       :", tl)
        print("  Tyreus-Luyben PI    :", tlpi)
        print("  No-overshoot        :", noovershoot)
        self.savetofile()

        chosen = znpi
        self.controller.setAllPID(chosen['Kp'], chosen['Ki'], chosen['Kd'])
        self.controller.setSP(self.targettemp)
        self.state = 'done'
        if self.oncomplete is not None:
            self.oncomplete(self.result)

    def savetofile(self):
        try:
            with open(self.resultsfile, 'a') as f:
                f.write(str(datetime.datetime.now()) + " target=" + str(self.targettemp)
                        + " " + str(self.result) + "\n")
            print("Results appended to", self.resultsfile)
        except OSError as e:
            print("Could not save autotune results:", e)

    def abort(self, reason):
        self.state = 'aborted'
        print("Relay autotune ABORTED:", reason)
        if self.oncomplete is not None:
            self.oncomplete(None)
