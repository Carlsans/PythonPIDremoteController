import time


class PIDOptimizer:
    """Online, gentle PID auto-tuner: nudges Kp/Ki/Kd very slowly based on
    the Integral of Absolute Error (IAE = sum of |setpoint - temp| over an
    evaluation window), instead of the relay method's large on/off swings.

    Runs as an add-on alongside a normal, already-holding pidprogram run -
    it never changes the setpoint, only the *cons* PID gains (the profile
    the firmware uses once close to target). One parameter (Kp, then Ki,
    then Kd, cycling) is nudged by a small relative step at the end of each
    window, in whichever direction the previous step's IAE change suggests
    (classic finite-difference coordinate descent - the same idea the
    project's original, since-removed OptimizePID.py attempted, done with a
    bounded relative step and an actual hard safety limit this time).

    Safety limit: `update()` is called every second, and if the measured
    temperature strays past `maxswing * safetymargin` from the live
    setpoint, tuning is not "slowly corrected" - the cons gains are
    reverted immediately to the last known-safe tunings (or, if no window
    has ever completed safely yet, backed off hard: Kp halved, Ki cut to
    30%) and a cooldown period is enforced before trying anything again.

    Why trip BELOW maxswing rather than exactly at it: heat already in
    flight (the pot's dead time + the element's own thermal capacity - the
    same "stored heat" effect measured on the real pot, where the water
    kept warming for minutes after the output was cut) keeps arriving for a
    while after the tunings are reverted, so the temperature can keep
    climbing briefly even after a correct, immediate reaction. The
    safetymargin (default 0.6, i.e. trip at 60% of maxswing) is the
    software's best-effort compensation for that physical lag - it is not
    a mathematical guarantee independent of the process's actual thermal
    dynamics, just a margin sized generously against the real pot's
    measured stored-heat behaviour (see TwoCapacityPot in tests/simulator.py
    and the tests in tests/test_pidoptimizer.py that exercise this).
    """

    # Fallback absolute step used when a relative step (value * stepfraction)
    # would be zero or negligible - most notably when a parameter starts at
    # exactly 0.0 (e.g. Kd, which defaults to 0 because of the MAX6675's
    # quantization - see PIDProgram.py). Without this, "value * (1 +
    # stepfraction)" computes 0 * anything = 0 forever, and that parameter
    # can never actually be explored. Found on a real run: Kd stayed
    # permanently stuck at 0.0 across repeated windows.
    DEFAULT_MINSTEPS = {'Kp': 0.5, 'Ki': 0.002, 'Kd': 0.1}

    def __init__(self, controller, starttunings, maxswing=3.0, safetymargin=0.6,
                 windowseconds=15 * 60, stepfraction=0.15,
                 windowsperparam=3, settleseconds=5 * 60, minsteps=None,
                 timesource=time.time, onstatechange=None):
        self.controller = controller
        self.maxswing = maxswing
        self.minsteps = dict(self.DEFAULT_MINSTEPS)
        if minsteps:
            self.minsteps.update(minsteps)
        self.tripthreshold = maxswing * safetymargin
        self.windowseconds = windowseconds
        self.stepfraction = stepfraction
        self.windowsperparam = windowsperparam
        # How long the temperature must stay continuously within
        # tripthreshold before the optimizer trusts it as "actually
        # settled" and starts a window. This matters because the aggressive
        # firmware profile (used whenever the gap is over 4.5 C) drives
        # near-full power during any approach, storing heat in the element
        # regardless of what the cons tunings are - that heat keeps
        # arriving for a while after the gap first closes, so engaging the
        # instant the gap crosses tripthreshold risks reacting to (or
        # measuring IAE against) an overshoot that is already in flight and
        # physically cannot be stopped by a tuning change. Requiring a
        # settled period first lets that stored heat dissipate before
        # tuning changes are trusted to mean anything.
        self.settleseconds = settleseconds
        self.now = timesource
        # Called with a short status string on notable events (window done,
        # safety trip, parameter change) - the GUI uses this to log/refresh.
        self.onstatechange = onstatechange

        self.paramorder = ['Kp', 'Ki', 'Kd']
        self.paramindex = 0
        self.cyclesonparam = 0

        self.safetunings = dict(starttunings)
        self.currenttunings = dict(starttunings)
        self.evercompletedwindow = False

        self.lastparamvalue = None
        self.lastiae = None

        # waiting -> running <-> cooldown, or 'stopped'. Starts in 'waiting'
        # so starting this during (or before) a cold approach is harmless:
        # a fresh approach legitimately has |temp-setpoint| > maxswing for a
        # while, which is normal and not something the safety trip should
        # ever react to - it only starts actually monitoring/tuning once
        # the temperature is already within maxswing of setpoint, matching
        # the real intended use ("retune while already holding").
        self.state = 'waiting'
        self.cooldownuntil = None
        self.withinrangesince = None
        self.windowstart = self.now()
        self.currentiae = 0.0
        self.currentmaxerr = 0.0
        self.samplecount = 0

        self.history = []  # each entry: dict describing a window or a trip
        self.applytunings(self.currenttunings)

    def notify(self, message):
        print("PIDOptimizer:", message)
        if self.onstatechange is not None:
            try:
                self.onstatechange(message)
            except Exception as e:
                print("PIDOptimizer onstatechange callback raised (ignored):", e)

    def applytunings(self, tunings):
        self.controller.setconsPIDvalues(tunings['Kp'], tunings['Ki'], tunings['Kd'])

    def currentparam(self):
        return self.paramorder[self.paramindex]

    def getprogress(self):
        elapsed = self.now() - self.windowstart
        return {
            "state": self.state,
            "param": self.currentparam(),
            "tunings": dict(self.currenttunings),
            "window_elapsed_s": elapsed,
            "window_seconds": self.windowseconds,
            "current_iae": self.currentiae,
            "current_maxerr": self.currentmaxerr,
            "windows_done": len(self.history),
            "cooldown_remaining_s": (max(0.0, self.cooldownuntil - self.now())
                                     if self.state == 'cooldown' and self.cooldownuntil else 0.0),
            "settling": self.withinrangesince is not None,
            "settle_remaining_s": (max(0.0, self.settleseconds - (self.now() - self.withinrangesince))
                                   if self.withinrangesince is not None else 0.0),
        }

    def stop(self):
        """Stop adapting; whatever tunings are currently applied stay
        active (the caller can read them via getprogress()['tunings'])."""
        self.state = 'stopped'
        self.notify("stopped - leaving current tunings in place: " + str(self.currenttunings))

    # ------------------------------------------------------------------
    def update(self, temp, setpoint=None, now=None):
        if self.state == 'stopped':
            return
        if now is None:
            now = self.now()
        if setpoint is None:
            setpoint = getattr(self.controller, 'SP', None)
        if temp is None or temp <= 0 or setpoint is None:
            return  # no valid reading yet

        error = abs(temp - setpoint)

        if self.state in ('waiting', 'cooldown'):
            if self.state == 'cooldown' and now < self.cooldownuntil:
                return  # still in the enforced cooldown window
            if error > self.tripthreshold:
                # Not in range (still approaching, or cooldown elapsed but
                # not actually recovered yet) - keep waiting. This does NOT
                # call tripsafety() again: repeatedly re-tripping while
                # merely still recovering would keep backing off the
                # tunings for no reason, since they are not what is keeping
                # the temperature out of range at this point.
                self.state = 'waiting'
                self.withinrangesince = None
                return
            # In range, but only trust it once it has STAYED in range for
            # settleseconds - see settleseconds' docstring for why (stored
            # heat from an aggressive-profile approach keeps arriving for a
            # while after the gap first closes, regardless of cons tunings).
            if self.withinrangesince is None:
                self.withinrangesince = now
            if now - self.withinrangesince < self.settleseconds:
                return
            wascooldown = self.state == 'cooldown'
            self.state = 'running'
            self.resetwindow(now)
            self.notify(("cooldown over, resuming" if wascooldown else "settled at target")
                       + " at " + str(self.currenttunings) + " - tuning " + self.currentparam())
            return

        if error > self.tripthreshold:
            self.tripsafety(temp, setpoint, now)
            return

        self.currentiae += error
        self.currentmaxerr = max(self.currentmaxerr, error)
        self.samplecount += 1

        if now - self.windowstart >= self.windowseconds:
            self.finishwindow(now)

    def resetwindow(self, now):
        self.windowstart = now
        self.currentiae = 0.0
        self.currentmaxerr = 0.0
        self.samplecount = 0

    def tripsafety(self, temp, setpoint, now):
        if self.evercompletedwindow:
            fallback = dict(self.safetunings)
            reason = "reverting to last known-safe tunings"
        else:
            # The STARTING tunings themselves are the problem (never even
            # made it through one window). Back off hard rather than
            # reverting to the same tunings that just failed.
            fallback = {'Kp': self.safetunings['Kp'] * 0.5,
                       'Ki': self.safetunings['Ki'] * 0.3,
                       'Kd': self.safetunings.get('Kd', 0.0)}
            self.safetunings = dict(fallback)
            reason = "starting tunings were already unsafe, backing off hard to"
        self.notify("SAFETY TRIP: |" + str(round(temp, 2)) + " - " + str(setpoint) + "| exceeded "
                    + str(self.tripthreshold) + " C (trip threshold; hard limit is " + str(self.maxswing)
                    + " C) - " + reason + " " + str(fallback))
        self.currenttunings = fallback
        self.applytunings(fallback)
        # Reducing Kp/Ki alone does NOT unwind an already-accumulated
        # integral term - the Arduino PID library (and this project's
        # firmware) keeps iterm as persistent state across SetTunings calls,
        # so a wound-up integral from the tunings that just tripped can keep
        # driving high output for a while even with the new, gentler gains.
        # The firmware's SetSP handler always resets the integral, even for
        # an unchanged value (main.cpp/OTAInputOutput.h calls
        # myPID.Initialize()+Reset() unconditionally) - re-sending the
        # current setpoint exploits that as a deliberate integral-reset
        # trick. Uses sendMessage() directly (not setSP()) because setSP()
        # skips sending entirely when the value has not changed, which is
        # exactly the case here.
        if hasattr(self.controller, 'sendMessage'):
            self.controller.sendMessage("SetSP(" + str(setpoint) + ")")
        # Reset the coordinate-descent memory: the interrupted window's
        # gradient estimate is not trustworthy.
        self.lastparamvalue = None
        self.lastiae = None
        self.state = 'cooldown'
        self.withinrangesince = None
        self.cooldownuntil = now + max(self.windowseconds / 3.0, 120.0)
        self.history.append({'event': 'safety_trip', 'temp': temp, 'setpoint': setpoint, 'time': now})

    def finishwindow(self, now):
        param = self.currentparam()
        value = self.currenttunings[param]
        iae = self.currentiae
        self.history.append({'event': 'window', 'param': param, 'value': value,
                             'iae': iae, 'maxerr': self.currentmaxerr, 'time': now})
        self.notify("window done: " + param + "=" + str(round(value, 5))
                    + " IAE=" + str(round(iae, 2)) + " maxerr=" + str(round(self.currentmaxerr, 3)))

        # This window stayed within the safety limit for its whole duration:
        # trust its tunings as the new fallback.
        self.evercompletedwindow = True
        self.safetunings = dict(self.currenttunings)

        minstep = self.minsteps.get(param, 0.01)
        if self.lastparamvalue is None:
            # First measurement for this parameter: take an initial step,
            # remember this point as the baseline for the next comparison.
            self.lastparamvalue = value
            self.lastiae = iae
            step = value * self.stepfraction
            if abs(step) < minstep:
                step = minstep
            newvalue = value + step
        else:
            deltaparam = value - self.lastparamvalue
            deltaiae = iae - self.lastiae
            slope = 0.0 if abs(deltaparam) < 1e-12 else deltaiae / deltaparam
            direction = -1.0 if slope > 0 else 1.0
            stepsize = max(abs(value) * self.stepfraction * 0.5, minstep * 0.5)
            newvalue = max(value + direction * stepsize, 0.0)
            self.lastparamvalue = value
            self.lastiae = iae

            self.cyclesonparam += 1
            if self.cyclesonparam >= self.windowsperparam:
                self.cyclesonparam = 0
                self.paramindex = (self.paramindex + 1) % len(self.paramorder)
                param = self.currentparam()
                newvalue = self.currenttunings[param]  # start the new parameter unchanged
                self.lastparamvalue = None
                self.lastiae = None
                self.notify("moving on to tuning " + param)

        self.currenttunings[param] = newvalue
        self.applytunings(self.currenttunings)
        self.resetwindow(now)
