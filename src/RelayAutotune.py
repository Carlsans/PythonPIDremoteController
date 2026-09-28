import datetime
import os
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

    Check log: every instance writes a dedicated, timestamped log file
    (self.checklogpath, never shared with another tuning run) recording
    every raw reading, every relay toggle, every completed cycle and - at
    the end - the full arithmetic behind Ku, Tu and every derived tuning
    rule, worked out with real numbers so it can be checked by hand without
    re-deriving anything or reading this source file. See _writeheader().
    """

    OUTPUT_SPAN = 255.0  # MCU PID output range
    RELAY_KP = 10000.0   # pure-P gain that saturates the MCU PID either way

    def __init__(self, controller, targettemp=40.0, hysteresis=0.15,
                 cyclestomeasure=4, skipcycles=1, maxsafetemp=60.0,
                 timeoutseconds=12 * 60 * 60, timesource=time.time,
                 resultsfile="autotune_results.txt", oncomplete=None,
                 checklogfile=None):
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

        # --- Check log setup. One file per task, never appended-to by a
        # later run, so old and new tuning attempts can never mix in one
        # file. Opened before anything else happens so even a construction-
        # time problem downstream has a chance of being visible in it.
        if checklogfile is None:
            logdir = os.path.join(os.getcwd(), "logs")
            try:
                os.makedirs(logdir, exist_ok=True)
            except OSError:
                logdir = os.getcwd()
            checklogfile = os.path.join(
                logdir, "pidtune_check_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                + "_target" + ("%g" % targettemp) + "C.log")
        self.checklogpath = checklogfile
        self._checklogfile = None
        try:
            self._checklogfile = open(self.checklogpath, 'a', buffering=1)
        except OSError as e:
            print("Could not open PID-tune check log", self.checklogpath, "(continuing without it):", e)
        self._writeheader()
        print("PID tuning check log:", self.checklogpath)

    # ------------------------------------------------------------------
    # Check log: a dedicated, human-readable, real-time audit trail
    # ------------------------------------------------------------------
    def checklog(self, message):
        """Append one timestamped line to this task's check log. Never
        allowed to raise - a logging problem must never be able to break
        the tune itself."""
        if self._checklogfile is None:
            return
        try:
            self._checklogfile.write(
                datetime.datetime.now().isoformat(sep=' ', timespec='milliseconds')
                + " | " + message + "\n")
        except Exception:
            pass

    def checkloghr(self, char='-', width=78):
        self.checklog(char * width)

    def close(self):
        """Flush and close this task's check log file. Idempotent - safe to
        call more than once (finish()/abort() both call it on every exit
        path)."""
        if self._checklogfile is not None:
            try:
                self._checklogfile.close()
            except OSError:
                pass
            self._checklogfile = None

    @staticmethod
    def _s(x, decimals=4):
        """Format a possibly-None number for the log: rounds floats to a
        sane number of decimals, passes anything else through as str()."""
        if x is None:
            return "?"
        if isinstance(x, float):
            return str(round(x, decimals))
        return str(x)

    def _writeheader(self):
        self.checkloghr('=')
        self.checklog("RELAY AUTOTUNE - PID TUNING CHECK LOG")
        self.checklog("Started: " + datetime.datetime.now().isoformat(sep=' ', timespec='seconds'))
        self.checkloghr('=')
        self.checklog("")
        self.checklog("This file records every raw reading and every relay-toggle/cycle event")
        self.checklog("used by the tuning algorithm in real time, and - once enough cycles are")
        self.checklog("measured - the FULL arithmetic behind Ku, Tu and every derived tuning")
        self.checklog("rule, with real numbers substituted into each formula, so the result can")
        self.checklog("be checked by hand without re-deriving anything or reading the source.")
        self.checklog("")
        self.checklog("PARAMETERS (as given to this tuning run):")
        self.checklog("  target temperature (targettemp)     = " + self._s(self.targettemp) + " C")
        self.checklog("    -> the temperature the relay oscillates around")
        self.checklog("  hysteresis                          = +/- " + self._s(self.hysteresis) + " C")
        self.checklog("    -> the relay only flips once temp crosses target +/- this amount, so")
        self.checklog("       sensor noise alone cannot cause chatter")
        self.checklog("  cycles to measure (cyclestomeasure) = " + self._s(self.cyclestomeasure))
        self.checklog("    -> number of full oscillation cycles averaged into the final result")
        self.checklog("  cycles to skip (skipcycles)         = " + self._s(self.skipcycles))
        self.checklog("    -> number of INITIAL cycles thrown away before averaging, since the")
        self.checklog("       oscillation has not settled into a steady pattern yet")
        self.checklog("  max safe temperature (maxsafetemp)  = " + self._s(self.maxsafetemp) + " C")
        self.checklog("    -> hard ceiling; the tune aborts immediately if temp ever exceeds this")
        self.checklog("  timeout (timeoutseconds)            = " + self._s(self.timeoutseconds)
                      + "s (" + self._s(self.timeoutseconds / 3600.0, 2) + " h)")
        self.checklog("    -> the tune aborts if this much time passes after the FIRST valid")
        self.checklog("       temperature reading without measuring enough cycles")
        self.checklog("  RELAY_KP (class constant)           = " + self._s(self.RELAY_KP))
        self.checklog("    -> the huge pure-P gain sent to the MCU's real PID during tuning, so")
        self.checklog("       its output saturates fully on or fully off depending only on the")
        self.checklog("       sign of (setpoint - measured temp) - this is what turns the MCU's")
        self.checklog("       own PID into an on/off relay")
        self.checklog("  OUTPUT_SPAN (class constant)        = " + self._s(self.OUTPUT_SPAN))
        self.checklog("    -> the MCU's nominal PID output range (0-255); used as a FALLBACK for")
        self.checklog("       the relay amplitude d only if the real output swing was never")
        self.checklog("       observed to move by more than 1.0 (see 'RELAY AMPLITUDE d' later)")
        self.checklog("")
        self.checklog("Relay logic: whenever OFF and temp <= target-hysteresis, the relay turns")
        self.checklog("ON (setpoint sent = min(target+15, maxsafetemp), i.e. heater full power).")
        self.checklog("Whenever ON and temp >= target+hysteresis, the relay turns OFF (setpoint")
        self.checklog("sent = 1, i.e. heater full off). Each OFF->ON transition starts a new")
        self.checklog("cycle; the peak and trough temperature seen during that cycle are what")
        self.checklog("the amplitude is computed from.")
        self.checkloghr('=')
        self.checklog("")
        self.checklog("LIVE READINGS (one line per update() call once the tune is running):")
        self.checklog("  t=<elapsed s>  temp=<C>  relay=<ON/OFF>  output=<raw MCU 0-255>")
        self.checklog("  cycle_max/cycle_min=<running peak/trough of the CURRENT cycle>")
        self.checklog("  observed_output_max/min=<running extremes of output for the WHOLE run>")
        self.checkloghr('-')
        self.checklog("")

    # ------------------------------------------------------------------
    # Relay actuation
    # ------------------------------------------------------------------
    def setrelaytunings(self):
        self.controller.setAllPID(self.RELAY_KP, 0.0, 0.0)
        self.checklog("Set MCU tunings to the relay profile: setAllPID(Kp=" + self._s(self.RELAY_KP)
                      + ", Ki=0, Kd=0) - pure P, huge gain, so output saturates fully on/off.")

    def relay(self, on):
        if on == self.relayon:
            return
        self.relayon = on
        if on:
            # Setpoint above target saturates the huge-Kp PID to full output,
            # but stays bounded so a dead controller cannot overheat the pot.
            sp = min(self.targettemp + 15, self.maxsafetemp)
            self.controller.setSP(sp)
            self.checklog("RELAY -> ON  (heater full power): setSP(min(target+15, maxsafetemp)) "
                          "= min(" + self._s(self.targettemp) + "+15, " + self._s(self.maxsafetemp)
                          + ") = " + self._s(sp))
        else:
            # Setpoint far below target -> output 0.
            self.controller.setSP(1)
            self.checklog("RELAY -> OFF (heater full off): setSP(1)")

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
            self.checklog("SAFETY ABORT: temp " + self._s(temp) + " > maxsafetemp " + self._s(self.maxsafetemp))
            self.abort("temperature " + str(temp) + " above safe maximum " + str(self.maxsafetemp))
            return

        if self.state == 'init':
            self.starttime = now
            self.setrelaytunings()
            self.relay(temp < self.targettemp)
            self.state = 'relay'
            self.checklog("TUNE STARTED at t=0 (now=" + self._s(now, 2) + "): first reading temp="
                          + self._s(temp) + " C, target=" + self._s(self.targettemp)
                          + " C -> relay starts " + ("ON" if temp < self.targettemp else "OFF"))
            print("Relay autotune started. Target =", self.targettemp,
                  "hysteresis = +/-", self.hysteresis)
            return

        if now - self.starttime > self.timeoutseconds:
            self.controller.setSP(1)
            self.checklog("TIMEOUT: elapsed " + self._s(now - self.starttime, 1) + "s > timeoutseconds "
                          + self._s(self.timeoutseconds) + "s, only " + str(len(self.cyclepeaks))
                          + " cycle(s) recorded so far.")
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

        self.checklog("t=" + self._s(now - self.starttime, 1) + "s  temp=" + self._s(temp)
                      + " C  relay=" + ("ON " if self.relayon else "OFF")
                      + "  output=" + self._s(output) + "  cycle_max=" + self._s(self.currentmax)
                      + " cycle_min=" + self._s(self.currentmin) + "  observed_output_max="
                      + self._s(self.observedmaxout) + " observed_output_min=" + self._s(self.observedminout))

        if self.relayon and temp >= self.targettemp + self.hysteresis:
            threshold = self.targettemp + self.hysteresis
            self.checklog("  -> temp " + self._s(temp) + " >= target " + self._s(self.targettemp)
                          + " + hysteresis " + self._s(self.hysteresis) + " = " + self._s(threshold)
                          + ": turning relay OFF")
            self.relay(False)
        elif not self.relayon and temp <= self.targettemp - self.hysteresis:
            threshold = self.targettemp - self.hysteresis
            self.checklog("  -> temp " + self._s(temp) + " <= target " + self._s(self.targettemp)
                          + " - hysteresis " + self._s(self.hysteresis) + " = " + self._s(threshold)
                          + ": cycle boundary, turning relay ON")
            self.oncyclestart(now)
            if self.state == 'relay':  # oncyclestart may have finished the tune
                self.relay(True)

    def oncyclestart(self, now):
        """An off->on edge: one full oscillation cycle just completed."""
        if self.switchtimes:
            self.cyclepeaks.append(self.currentmax)
            self.cycletroughs.append(self.currentmin)
            period = now - self.switchtimes[-1]
            cyclenum = len(self.cyclepeaks)
            self.checkloghr()
            willskip = cyclenum <= self.skipcycles
            self.checklog("CYCLE " + str(cyclenum) + " COMPLETE"
                          + (" - will be SKIPPED as a settling cycle" if willskip
                             else " - will be USED in the average"))
            self.checklog("  cycle start (switchtimes[" + str(cyclenum - 1) + "]) = "
                          + self._s(self.switchtimes[-1], 2) + "s")
            self.checklog("  cycle end   (this edge, now)          = " + self._s(now, 2) + "s")
            self.checklog("  period = end - start = " + self._s(now, 2) + " - "
                          + self._s(self.switchtimes[-1], 2) + " = " + self._s(period, 2) + "s")
            self.checklog("  peak temp this cycle (cyclepeaks[" + str(cyclenum - 1) + "])    = "
                          + self._s(self.currentmax) + " C")
            self.checklog("  trough temp this cycle (cycletroughs[" + str(cyclenum - 1) + "]) = "
                          + self._s(self.currentmin) + " C")
            if self.currentmax is not None and self.currentmin is not None:
                amp = (self.currentmax - self.currentmin) / 2.0
                self.checklog("  amplitude = (peak - trough) / 2 = (" + self._s(self.currentmax) + " - "
                              + self._s(self.currentmin) + ") / 2 = " + self._s(amp))
            self.checkloghr()
            print("Autotune cycle " + str(len(self.cyclepeaks)) + " done: period=" + str(round(period, 1))
                  + "s peak=" + str(self.currentmax) + " trough=" + str(self.currentmin))
        self.switchtimes.append(now)
        self.currentmax = None
        self.currentmin = None
        if len(self.cyclepeaks) >= self.skipcycles + self.cyclestomeasure:
            self.checklog(str(self.skipcycles) + " skip + " + str(self.cyclestomeasure)
                          + " measure = " + str(self.skipcycles + self.cyclestomeasure)
                          + " cycles reached -> computing final tunings.")
            self.finish()

    def finish(self):
        """Full, successful completion: the configured number of cycles was
        measured. Compute tunings, push them to the controller and hold the
        target. If the final measurement is somehow degenerate (e.g. a flat
        sensor reading), fall back to abort() so any earlier partial data
        still isn't wasted."""
        if not self.computetunings():
            self.abort("degenerate oscillation in the final measurement")
            return
        chosen = self.result[self.result['applied']]
        self.controller.setAllPID(chosen['Kp'], chosen['Ki'], chosen['Kd'])
        self.controller.setSP(self.targettemp)
        self.checklog("Pushed to controller: setAllPID(Kp=" + self._s(chosen['Kp']) + ", Ki="
                      + self._s(chosen['Ki']) + ", Kd=" + self._s(chosen['Kd']) + "); setSP("
                      + self._s(self.targettemp) + ")")
        self.checklog("Check log closed: " + datetime.datetime.now().isoformat(sep=' ', timespec='seconds'))
        self.state = 'done'
        if self.oncomplete is not None:
            self.oncomplete(self.result)
        self.close()

    # ------------------------------------------------------------------
    # Tuning computation
    # ------------------------------------------------------------------
    def computetunings(self, extra=None):
        """Compute Ku/Tu and the derived tunings from whatever complete
        cycles are available beyond skipcycles - not necessarily the full
        cyclestomeasure count. Sets self.result and returns True if there
        was at least one usable cycle; returns False (leaving self.result
        untouched) otherwise. Callers decide what a partial vs a full result
        means (see finish() and abort()); this method has no side effects on
        self.state or the controller, so it is safe to call speculatively.

        Every step is also written out to the check log with the real
        numbers substituted into each formula (see the module docstring).
        """
        peaks = self.cyclepeaks[self.skipcycles:]
        troughs = self.cycletroughs[self.skipcycles:]
        periods = []
        for i in range(len(self.switchtimes) - 1):
            periods.append(self.switchtimes[i + 1] - self.switchtimes[i])
        periods = periods[self.skipcycles:]

        self.checkloghr('=')
        self.checklog("COMPUTING TUNINGS"
                      + (" (" + extra['aborted_reason'] + ")" if extra and extra.get('aborted_reason') else ""))
        self.checkloghr('=')
        self.checklog(str(len(self.cyclepeaks)) + " total cycle(s) recorded; skipping the first "
                      + str(self.skipcycles) + " as settling, leaving " + str(len(peaks))
                      + " cycle(s) to average (cyclestomeasure target = " + str(self.cyclestomeasure) + ").")
        self.checklog("")

        if not peaks or not troughs or not periods:
            self.checklog("NOT ENOUGH DATA: 0 cycles remain after skipping " + str(self.skipcycles)
                          + " - nothing can be computed from this run.")
            self.checkloghr('=')
            return False

        self.checklog("CYCLES USED (cycle# is the ORIGINAL cycle number, before skipping):")
        self.checklog("  cycle#   period(s)   peak(C)   trough(C)   amplitude(C)")
        amplitudes = []
        for i, (p, t, per) in enumerate(zip(peaks, troughs, periods)):
            amp = (p - t) / 2.0
            amplitudes.append(amp)
            self.checklog("  " + str(i + 1 + self.skipcycles).rjust(6) + "   "
                          + self._s(per, 2).rjust(9) + "   " + self._s(p).rjust(7) + "   "
                          + self._s(t).rjust(9) + "   " + self._s(amp).rjust(12))
        self.checklog("")

        Tu = sum(periods) / len(periods)
        self.checklog("ULTIMATE PERIOD Tu (average oscillation period):")
        self.checklog("  Tu = sum(periods) / count")
        self.checklog("     = (" + " + ".join(self._s(p, 2) for p in periods) + ") / " + str(len(periods)))
        self.checklog("     = " + self._s(sum(periods), 2) + " / " + str(len(periods))
                      + " = " + self._s(Tu, 6) + " s  (" + self._s(Tu / 60.0, 2) + " min)")
        self.checklog("")

        a = sum(amplitudes) / len(amplitudes)
        self.checklog("OSCILLATION AMPLITUDE a (average half peak-to-trough swing):")
        self.checklog("  a = sum(amplitudes) / count")
        self.checklog("    = (" + " + ".join(self._s(x, 4) for x in amplitudes) + ") / " + str(len(amplitudes)))
        self.checklog("    = " + self._s(sum(amplitudes), 4) + " / " + str(len(amplitudes))
                      + " = " + self._s(a, 6) + " C")
        self.checklog("")

        if a <= 0 or Tu <= 0:
            self.checklog("DEGENERATE RESULT: amplitude=" + self._s(a) + ", period=" + self._s(Tu)
                          + " - both must be positive to compute Ku. Aborting this computation"
                          + " (any EARLIER cycles already measured are unaffected).")
            self.checkloghr('=')
            return False

        usedmeasuredswing = (self.observedmaxout is not None and self.observedminout is not None
                             and self.observedmaxout - self.observedminout > 1.0)
        self.checklog("RELAY AMPLITUDE d (half the heater output's actual swing):")
        self.checklog("  observed output max = " + self._s(self.observedmaxout) + "  (raw MCU units, 0-255 scale)")
        self.checklog("  observed output min = " + self._s(self.observedminout))
        if usedmeasuredswing:
            swing = self.observedmaxout - self.observedminout
            d = swing / 2.0
            self.checklog("  swing = max - min = " + self._s(self.observedmaxout) + " - "
                          + self._s(self.observedminout) + " = " + self._s(swing))
            self.checklog("  swing > 1.0, so using the MEASURED swing (not the nominal OUTPUT_SPAN) -")
            self.checklog("  this matters because the real firmware may clamp its output below 255")
            self.checklog("  (this hardware is known to clamp near 160); using the nominal 255 when")
            self.checklog("  the real ceiling is lower would overstate Ku and give hotter tunings")
            self.checklog("  than the hardware can actually deliver.")
            self.checklog("  d = swing / 2 = " + self._s(swing) + " / 2 = " + self._s(d, 6))
        else:
            d = self.OUTPUT_SPAN / 2.0
            self.checklog("  swing was never observed to exceed 1.0 - falling back to the NOMINAL")
            self.checklog("  OUTPUT_SPAN. This usually means controller.currentoutput was never")
            self.checklog("  updated during this run (check the caller is feeding real 'Output:'")
            self.checklog("  messages in) - if so, this d (and therefore Ku) is NOT trustworthy.")
            self.checklog("  d = OUTPUT_SPAN / 2 = " + self._s(self.OUTPUT_SPAN) + " / 2 = " + self._s(d, 6))
        self.checklog("")

        pi = 3.141592653589793
        Ku = 4.0 * d / (pi * a)
        self.checklog("ULTIMATE GAIN Ku (Astrom-Hagglund relay-feedback formula):")
        self.checklog("  Ku = 4*d / (pi*a)")
        self.checklog("     = 4 * " + self._s(d, 6) + " / (3.14159265... * " + self._s(a, 6) + ")")
        self.checklog("     = " + self._s(4.0 * d, 6) + " / " + self._s(pi * a, 6))
        self.checklog("     = " + self._s(Ku, 6))
        self.checklog("")

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

        self.checklog("DERIVED TUNINGS (Ku=" + self._s(Ku, 6) + ", Tu=" + self._s(Tu, 6)
                      + "s in every formula below):")
        self.checklog("")
        self.checklog("  Ziegler-Nichols (classic PID):")
        self.checklog("    Kp = 0.6 * Ku         = 0.6 * " + self._s(Ku, 6) + " = " + self._s(zn['Kp'], 6))
        self.checklog("    Ki = 1.2 * Ku / Tu    = 1.2 * " + self._s(Ku, 6) + " / " + self._s(Tu, 6)
                      + " = " + self._s(zn['Ki'], 6))
        self.checklog("    Kd = 0.075 * Ku * Tu  = 0.075 * " + self._s(Ku, 6) + " * " + self._s(Tu, 6)
                      + " = " + self._s(zn['Kd'], 6))
        self.checklog("")
        self.checklog("  Ziegler-Nichols PI (no derivative):")
        self.checklog("    Kp = 0.45 * Ku        = 0.45 * " + self._s(Ku, 6) + " = " + self._s(znpi['Kp'], 6))
        self.checklog("    Ki = 0.54 * Ku / Tu   = 0.54 * " + self._s(Ku, 6) + " / " + self._s(Tu, 6)
                      + " = " + self._s(znpi['Ki'], 6))
        self.checklog("    Kd = 0  (fixed - no derivative term)")
        self.checklog("")
        self.checklog("  Tyreus-Luyben (classic PID):")
        self.checklog("    Kp = 0.454 * Ku           = 0.454 * " + self._s(Ku, 6) + " = " + self._s(tl['Kp'], 6))
        self.checklog("    Ki = 0.454*Ku/(2.2*Tu)    = 0.454 * " + self._s(Ku, 6) + " / (2.2 * "
                      + self._s(Tu, 6) + ") = " + self._s(tl['Ki'], 6))
        self.checklog("    Kd = 0.454*Ku*Tu/6.3      = 0.454 * " + self._s(Ku, 6) + " * " + self._s(Tu, 6)
                      + " / 6.3 = " + self._s(tl['Kd'], 6))
        self.checklog("")
        self.checklog("  Tyreus-Luyben PI (no derivative):")
        self.checklog("    Kp = Ku / 3.2             = " + self._s(Ku, 6) + " / 3.2 = " + self._s(tlpi['Kp'], 6))
        self.checklog("    Ki = Ku/(3.2*2.2*Tu)      = " + self._s(Ku, 6) + " / (3.2 * 2.2 * "
                      + self._s(Tu, 6) + ") = " + self._s(tlpi['Ki'], 6))
        self.checklog("    Kd = 0  (fixed)")
        self.checklog("")
        self.checklog("  No-overshoot (most conservative rule offered):")
        self.checklog("    Kp = 0.2 * Ku          = 0.2 * " + self._s(Ku, 6) + " = " + self._s(noovershoot['Kp'], 6))
        self.checklog("    Ki = 0.4 * Ku / Tu     = 0.4 * " + self._s(Ku, 6) + " / " + self._s(Tu, 6)
                      + " = " + self._s(noovershoot['Ki'], 6))
        self.checklog("    Kd = 0.0667*Ku*Tu      = 0.0667 * " + self._s(Ku, 6) + " * " + self._s(Tu, 6)
                      + " = " + self._s(noovershoot['Kd'], 6))
        self.checklog("")
        self.checklog("APPLIED RULE: ziegler_nichols_pi -> Kp=" + self._s(znpi['Kp'], 6) + " Ki="
                      + self._s(znpi['Ki'], 6) + " Kd=0")
        self.checklog("  This is a FIXED CHOICE (not re-evaluated per run): past hand-tuning on")
        self.checklog("  this hardware (Kp~20, Ki~0.02 at a low setpoint) sits closest to this")
        self.checklog("  rule's output of the 5 above, and Tyreus-Luyben's integral proved too")
        self.checklog("  slow in a real hold-temperature test on this pot (8.7 C droop over 10 min")
        self.checklog("  at 82 C). All 5 rules' numbers are given above so this choice can be")
        self.checklog("  second-guessed against the actual result of THIS run, not just the 82 C")
        self.checklog("  one that motivated it.")
        self.checklog("")

        self.result = {
            'Ku': Ku, 'Tu': Tu, 'amplitude': a, 'relay_amplitude': d,
            'ziegler_nichols': zn,
            'ziegler_nichols_pi': znpi,
            'tyreus_luyben': tl,
            'tyreus_luyben_pi': tlpi,
            'no_overshoot': noovershoot,
            'applied': 'ziegler_nichols_pi',
            'cycles_used': len(amplitudes),
            'cycles_target': self.cyclestomeasure,
        }
        if extra:
            self.result.update(extra)
        complete = len(amplitudes) >= self.cyclestomeasure
        self.checklog("RESULT: " + ("FULL run" if complete else "PARTIAL run") + " - "
                      + str(len(amplitudes)) + "/" + str(self.cyclestomeasure) + " target cycle(s) used.")
        if extra and extra.get('aborted_reason'):
            self.checklog("  Stopped early: " + extra['aborted_reason'])
        self.checkloghr('=')
        self.checklog("")

        print("Relay autotune " + ("finished" if complete else "computed PARTIAL tunings")
              + " (" + str(len(amplitudes)) + "/" + str(self.cyclestomeasure) + " cycle(s) measured).")
        print("  Ultimate gain Ku =", Ku, " ultimate period Tu =", Tu, "s, amplitude =", a)
        print("  Ziegler-Nichols     :", zn)
        print("  Ziegler-Nichols PI  :", znpi, "(applied)")
        print("  Tyreus-Luyben       :", tl)
        print("  Tyreus-Luyben PI    :", tlpi)
        print("  No-overshoot        :", noovershoot)
        self.savetofile()
        return True

    def savetofile(self):
        try:
            with open(self.resultsfile, 'a') as f:
                f.write(str(datetime.datetime.now()) + " target=" + str(self.targettemp)
                        + " " + str(self.result) + "\n")
            print("Results appended to", self.resultsfile)
        except OSError as e:
            print("Could not save autotune results:", e)

    def getprogress(self):
        """Snapshot of where the autotune is, for display in the GUI."""
        elapsed = 0.0 if self.starttime is None else self.now() - self.starttime
        return {"phase": self.state,
                "elapsed_s": elapsed,
                "cycles_done": len(self.cyclepeaks),
                "cycles_needed": self.skipcycles + self.cyclestomeasure,
                "target": self.targettemp}

    def abort(self, reason):
        """Stop the tune early (timeout, safety limit, or a caller asking to
        stop - e.g. the GUI's Stop button). Relay autotune is slow (tens of
        minutes per cycle), so rather than discarding everything, compute
        tunings from whatever complete cycles were already measured beyond
        skipcycles - even just one - and report them via oncomplete() as a
        'partial' result (result['cycles_used'] < result['cycles_target']).
        oncomplete(None) only happens when there is truly nothing usable."""
        if self.state in ('done', 'aborted'):
            return  # already finished/aborted - never double-report
        self.state = 'aborted'
        self.checkloghr('#')
        self.checklog("ABORT: " + reason)
        self.checkloghr('#')
        if self.computetunings(extra={'aborted_reason': reason}):
            print("Relay autotune ABORTED (" + reason + ") but " + str(self.result['cycles_used'])
                  + "/" + str(self.result['cycles_target']) + " cycle(s) were measured - partial "
                  "tunings were computed (result['cycles_used'], result['aborted_reason']). These "
                  "were NOT pushed to the controller automatically - review before trusting them fully.")
            self.checklog("Check log closed: " + datetime.datetime.now().isoformat(sep=' ', timespec='seconds'))
            if self.oncomplete is not None:
                self.oncomplete(self.result)
            self.close()
            return
        print("Relay autotune ABORTED:", reason, "- no usable cycles were measured, nothing computed.")
        self.checklog("Nothing was computed - 0 usable cycles.")
        self.checklog("Check log closed: " + datetime.datetime.now().isoformat(sep=' ', timespec='seconds'))
        if self.oncomplete is not None:
            self.oncomplete(None)
        self.close()
