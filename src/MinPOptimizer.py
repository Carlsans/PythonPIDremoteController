import time


class MinPOptimizer:
    """Online, gentle PID auto-tuner using a different strategy than
    PIDOptimizer's IAE gradient descent - which, tried live on the real
    fermenter, never converged: the SAME Kp scored IAE=125.82 in one window
    and IAE=379.41 in another, meaning real process noise/disturbances
    (fermentation heat, ambient swings) were swamping the small
    window-to-window tuning signal the gradient estimate depends on.

    Strategy instead: minimize Kp as far as it will go, and only introduce
    just enough Ki to keep the temperature holding once P alone can no
    longer do it. On a slow, thermal-mass-heavy process driven through a
    quantized sensor (MAX6675) and a coarse actuator, a high Kp mostly adds
    oscillation/noise amplification; I is what actually cancels the
    steady heat-loss-driven offset. This is a monotonic, one-direction-at-
    a-time search rather than a gradient estimate, which only ever asks a
    much more robust question: "did the temperature stay close to target
    for a whole window, yes or no" - not "was the IAE a little better or a
    little worse than last time," which is exactly the comparison real
    process noise kept corrupting.

    Two-phase, single pass:

    1. lowering_p - Kp is stepped down (Ki/Kd held fixed) each window it
       stays within `driftthreshold` of setpoint on average. As soon as a
       window's mean error exceeds `driftthreshold` (the process could no
       longer hold itself at that Kp), Kp is stepped back up to the last
       value that held, and phase 2 begins. If Kp reaches exactly 0.0 and
       still holds, the search stops there - nothing left to minimize.
    2. raising_i - Ki is stepped up (Kp/Kd held fixed) each window until
       the mean error stays under `holdthreshold` for `confirmwindows`
       consecutive windows, at which point the search stops (state becomes
       'holding') and tunings are frozen at whatever was found.

    Safety limit: identical hard guarantee to PIDOptimizer - update() is
    called every second, and if the measured temperature ever strays past
    `maxswing * safetymargin` from the live setpoint, tunings are reverted
    immediately to the last known-safe tunings (or backed off hard if none
    have ever proven safe yet), the firmware's integral is force-reset via
    a SetSP resend, and a cooldown is enforced before resuming the same
    phase it was in. See PIDOptimizer's docstring for why the trip
    threshold sits below maxswing (stored heat already in flight keeps
    arriving briefly even after a correct, immediate reaction) - that
    reasoning applies identically here.
    """

    DEFAULT_KIMINSTEP = 0.002

    def __init__(self, controller, starttunings, maxswing=3.0, safetymargin=0.6,
                 windowseconds=15 * 60, settleseconds=5 * 60,
                 pstepfraction=0.2, pfloorsnap=1.0,
                 kistepfraction=0.25, kiminstep=None,
                 driftthreshold=0.5, holdthreshold=0.2, confirmwindows=2,
                 timesource=time.time, onstatechange=None):
        self.controller = controller
        self.maxswing = maxswing
        self.tripthreshold = maxswing * safetymargin
        self.windowseconds = windowseconds
        self.settleseconds = settleseconds
        self.pstepfraction = pstepfraction
        # Below this Kp, snap straight to 0.0 instead of asymptotically
        # approaching it forever via repeated multiplication.
        self.pfloorsnap = pfloorsnap
        self.kistepfraction = kistepfraction
        self.kiminstep = kiminstep if kiminstep is not None else self.DEFAULT_KIMINSTEP
        self.driftthreshold = driftthreshold
        self.holdthreshold = holdthreshold
        self.confirmwindows = confirmwindows
        self.now = timesource
        # Called with a short status string on notable events (window done,
        # phase change, safety trip) - the GUI uses this to log/refresh.
        self.onstatechange = onstatechange

        self.safetunings = dict(starttunings)
        self.currenttunings = dict(starttunings)
        self.evercompletedwindow = False
        self.lastholdingkp = starttunings['Kp']
        self.consecutiveholding = 0

        # waiting -> lowering_p -> raising_i -> holding, with 'cooldown'
        # reachable from any active phase on a safety trip (resumes the
        # same phase afterward), or 'stopped' once stop() is called.
        self.state = 'waiting'
        self.phaseaftercooldown = 'lowering_p'
        self.cooldownuntil = None
        self.withinrangesince = None
        self.windowstart = self.now()
        self.errorsum = 0.0
        self.currentmaxerr = 0.0
        self.samplecount = 0

        self.history = []  # each entry: dict describing a window or a trip
        self.applytunings(self.currenttunings)

    def notify(self, message):
        print("MinPOptimizer:", message)
        if self.onstatechange is not None:
            try:
                self.onstatechange(message)
            except Exception as e:
                print("MinPOptimizer onstatechange callback raised (ignored):", e)

    def applytunings(self, tunings):
        self.controller.setconsPIDvalues(tunings['Kp'], tunings['Ki'], tunings.get('Kd', 0.0))

    def getprogress(self):
        elapsed = self.now() - self.windowstart
        meanerror = 0.0 if self.samplecount == 0 else self.errorsum / self.samplecount
        return {
            "state": self.state,
            "tunings": dict(self.currenttunings),
            "window_elapsed_s": elapsed,
            "window_seconds": self.windowseconds,
            "current_meanerror": meanerror,
            "current_maxerr": self.currentmaxerr,
            "windows_done": len([e for e in self.history if e.get('event') == 'window']),
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

        error = temp - setpoint  # signed: positive = running hot, negative = running cold
        abserror = abs(error)

        if self.state in ('waiting', 'cooldown'):
            if self.state == 'cooldown' and now < self.cooldownuntil:
                return
            if abserror > self.tripthreshold:
                self.state = 'waiting'
                self.withinrangesince = None
                return
            if self.withinrangesince is None:
                self.withinrangesince = now
            if now - self.withinrangesince < self.settleseconds:
                return
            wascooldown = self.state == 'cooldown'
            self.state = self.phaseaftercooldown
            self.resetwindow(now)
            self.notify(("cooldown over, resuming " if wascooldown else "settled at target, starting ")
                       + self.state + " at " + str(self.currenttunings))
            return

        if self.state == 'holding':
            # Keep monitoring for safety, but no more active tuning.
            if abserror > self.tripthreshold:
                self.tripsafety(temp, setpoint, now, resumephase='holding')
            return

        # state is 'lowering_p' or 'raising_i'
        if abserror > self.tripthreshold:
            self.tripsafety(temp, setpoint, now, resumephase=self.state)
            return

        self.errorsum += error
        self.currentmaxerr = max(self.currentmaxerr, abserror)
        self.samplecount += 1

        if now - self.windowstart >= self.windowseconds:
            self.finishwindow(now)

    def resetwindow(self, now):
        self.windowstart = now
        self.errorsum = 0.0
        self.currentmaxerr = 0.0
        self.samplecount = 0

    def tripsafety(self, temp, setpoint, now, resumephase):
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
                    + str(round(self.tripthreshold, 3)) + " C (trip threshold; hard limit is " + str(self.maxswing)
                    + " C) - " + reason + " " + str(fallback))
        self.currenttunings = fallback
        self.applytunings(fallback)
        # Same integral-reset-via-resend trick as PIDOptimizer - see its
        # tripsafety() docstring for the full reasoning (firmware's SetSP
        # handler unconditionally resets the accumulated integral term).
        if hasattr(self.controller, 'sendMessage'):
            self.controller.sendMessage("SetSP(" + str(setpoint) + ")")
        self.consecutiveholding = 0
        self.state = 'cooldown'
        self.phaseaftercooldown = resumephase
        self.withinrangesince = None
        self.cooldownuntil = now + max(self.windowseconds / 3.0, 120.0)
        self.history.append({'event': 'safety_trip', 'temp': temp, 'setpoint': setpoint, 'time': now})

    def finishwindow(self, now):
        meanerror = self.errorsum / self.samplecount if self.samplecount else 0.0
        phase = self.state
        self.history.append({'event': 'window', 'phase': phase, 'tunings': dict(self.currenttunings),
                             'meanerror': meanerror, 'maxerr': self.currentmaxerr, 'time': now})
        self.notify("window done (" + phase + "): " + str({k: round(v, 5) for k, v in self.currenttunings.items()})
                    + " meanerror=" + str(round(meanerror, 3)) + " maxerr=" + str(round(self.currentmaxerr, 3)))

        # This window stayed within the HARD safety limit for its whole
        # duration (a softer drift check below is what judges hold
        # quality) - trust it as the new safe fallback.
        self.evercompletedwindow = True
        self.safetunings = dict(self.currenttunings)

        if phase == 'lowering_p':
            if abs(meanerror) <= self.driftthreshold:
                kp = self.currenttunings['Kp']
                if kp <= 1e-9:
                    # Already at true zero and still holding: nothing left
                    # to minimize on the P side.
                    self.notify("Kp already at 0 and still holding - stopping the search")
                    self.state = 'holding'
                    self.consecutiveholding = self.confirmwindows
                else:
                    self.lastholdingkp = kp
                    newkp = kp * (1.0 - self.pstepfraction)
                    if newkp < self.pfloorsnap:
                        newkp = 0.0
                    self.currenttunings['Kp'] = newkp
                    self.applytunings(self.currenttunings)
            else:
                self.notify("Kp=" + str(round(self.currenttunings['Kp'], 4))
                            + " could no longer hold (meanerror=" + str(round(meanerror, 3))
                            + " C) - backing Kp off to " + str(round(self.lastholdingkp, 4))
                            + " and switching to raising Ki")
                self.currenttunings['Kp'] = self.lastholdingkp
                self.safetunings = dict(self.currenttunings)
                self.applytunings(self.currenttunings)
                self.state = 'raising_i'
                self.consecutiveholding = 0
        elif phase == 'raising_i':
            if abs(meanerror) <= self.holdthreshold:
                self.consecutiveholding += 1
                if self.consecutiveholding >= self.confirmwindows:
                    self.notify("holding restored at " + str(self.currenttunings) + " - search complete")
                    self.state = 'holding'
            else:
                self.consecutiveholding = 0
                ki = self.currenttunings['Ki']
                step = max(ki * self.kistepfraction, self.kiminstep)
                self.currenttunings['Ki'] = ki + step
                self.applytunings(self.currenttunings)

        self.resetwindow(now)
