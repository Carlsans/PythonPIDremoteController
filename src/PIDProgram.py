import os
import time
import chime

class PIDProgram:
    """Runs a staged temperature program: for each stage, heat to the stage
    temperature, then hold it for the stage duration before moving on.

    `stages` is a list of dicts: {"temperature": C, "duration_minutes": min,
    "override_pid": bool (optional), "fast_approach": bool (optional)}. When
    omitted, the historical default (39 C held for 24 h) is used.

    `fast_approach` (default False) skips the setpoint ramp for that stage's
    approach (see updateapproachramp()): the full target is sent immediately,
    so the firmware's aggressive (full-power) profile drives the climb the
    whole way instead of being capped to trickle in behind the measured
    temperature. Faster, at the cost of the stored-heat overshoot the ramp
    exists to prevent - useful for a stage like an 82 C sanitizing hold where
    a few degrees of overshoot are harmless but a slow approach is not worth
    the wait, unlike a stage holding near a live, temperature-sensitive
    culture.

    `stagetunings`, if given, is a list of (Kp, Ki, Kd) tuples aligned with
    `stages` - a stage's own entry (when not None) is applied as the cons
    profile whenever that stage becomes current (at program start and on
    every stage transition), letting each stage of a program use a
    different PID profile (e.g. a stiffer one for an 82 C sanitizing stage,
    a gentler one for a 38 C hold). A stage with no entry (None) falls back
    to `tunings`, the single profile used before this existed.
    """
    def __init__(self,controller, stages=None, tunings=None, stagetunings=None, timesource=time.time,
                 approachtunings=(100.0, 0.0, 0.0),
                 ramprate=0.5 / 60.0, rampzone=10.0):
        self.controller = controller
        self.now = timesource
        self.programstarttime = self.now()
        if tunings is None:
            tunings = (20, .02, .4)  # "medium-pot-5-jars" profile
        self.tunings = tunings
        self.stagetunings = list(stagetunings) if stagetunings else []
        # The firmware uses the aggressive profile while |SP - temp| >= 4.5 C
        # and the conservative one when closer. The profile the user tuned
        # goes in cons (it holds the temperature); agg gets pure P with Ki=0
        # so the integral cannot wind up during long climbs - windup there is
        # what caused massive overshoot when heating to a low setpoint like
        # 38 C, where the pot barely cools passively. (Setting both profiles
        # to the same PI, as setAllPID did, defeats this firmware feature.)
        self.controller.setaggPIDvalues(approachtunings[0], approachtunings[1], approachtunings[2])
        try:
            chime.theme('zelda')
        except Exception as e:
            print("Chime unavailable (ignored):", e)

        self.startnextsteptime = self.now()
        if stages is None:
            stages = [{"temperature": 39.0, "duration_minutes": 24 * 60}]
        if not stages:
            raise ValueError("The program needs at least one stage.")
        self.temperatures = [float(s["temperature"]) for s in stages]
        self.waitsecondsaftertempreached = [float(s["duration_minutes"]) * 60 for s in stages]
        self.temperaturetolerance = 0.3

        if os.environ.get('YOGURT_SILENT'):
            self.effects = ["none" for i in self.waitsecondsaftertempreached]
        else:
            self.effects = ["chime" for i in self.waitsecondsaftertempreached]
        self.overridepid = [bool(s.get("override_pid", False)) for s in stages]
        self.fastapproach = [bool(s.get("fast_approach", False)) for s in stages]

        # Setpoint ramp for upward approaches. The pot stores a lot of heat in
        # its element/bottom during a full-power climb (~8 C worth on the real
        # pot) which keeps flowing into the water for minutes after the output
        # cuts - the sensor cannot see it, so no PID tuning can prevent the
        # overshoot. Instead: full power until `rampzone` C below the target,
        # then ramp the setpoint up at `ramprate` C/s so the stored heat is
        # delivered *during* the approach instead of after it.
        self.ramprate = ramprate
        self.rampzone = rampzone
        self.rampstart = None
        self.rampstartsp = None

        self.currentstage = 0
        self.temperaturehasbeenreached = False
        self.endannounced = False
        # Pré-initialization. setSP() pushes the setpoint to the MCU right
        # away instead of waiting for the next SetPoint-echo resync.
        self.controller.setSP(self.temperatures[self.currentstage])
        self.controller.overridepid = self.overridepid[self.currentstage]
        if self.overridepid[self.currentstage]:
            self.controller.setPIDOverride()
        self.applystagetunings(self.currentstage)

    def applystagetunings(self, stageindex):
        """Apply the cons profile for `stageindex`: its own tunings if the
        stage specifies one, otherwise the program's fallback `tunings`."""
        stagespecific = self.stagetunings[stageindex] if stageindex < len(self.stagetunings) else None
        kp, ki, kd = stagespecific if stagespecific else self.tunings
        self.controller.setconsPIDvalues(kp, ki, kd)

    def getprogress(self):
        """Snapshot of where the program is, for display in the GUI.

        `stage_remaining_s`/`total_remaining_s` only count hold time (once a
        stage's target is reached, how long until it moves on); the time
        spent heating up to a target is not predictable (it depends on the
        pot, the ambient temperature, how full it is...), so it is not
        included and the "heating" phase is shown instead as open-ended.
        """
        totalelapsed = self.now() - self.programstarttime
        ended = not self.currentstage <= len(self.temperatures) - 1
        if ended:
            return {"ended": True, "phase": "ended",
                    "stage_index": len(self.temperatures) - 1,
                    "stage_count": len(self.temperatures),
                    "target": self.temperatures[-1] if self.temperatures else None,
                    "total_elapsed_s": totalelapsed,
                    "stage_elapsed_s": None, "stage_remaining_s": 0.0,
                    "total_remaining_s": 0.0}
        stageduration = self.waitsecondsaftertempreached[self.currentstage]
        if self.temperaturehasbeenreached:
            phase = "holding"
            stageremaining = max(0.0, self.startnextsteptime - self.now())
            stageelapsed = stageduration - stageremaining
        else:
            phase = "heating"
            stageremaining = stageduration
            stageelapsed = 0.0
        futureholds = sum(self.waitsecondsaftertempreached[self.currentstage + 1:])
        return {"ended": False, "phase": phase,
                "stage_index": self.currentstage,
                "stage_count": len(self.temperatures),
                "target": self.temperatures[self.currentstage],
                "total_elapsed_s": totalelapsed,
                "stage_elapsed_s": stageelapsed, "stage_remaining_s": stageremaining,
                "total_remaining_s": stageremaining + futureholds}

    def playchime(self, sound):
        try:
            sound()
        except Exception as e:
            # A missing audio device must never stop the fermentation program.
            print("Could not play chime (ignored):", e)

    def updateapproachramp(self):
        """Manage the ramped setpoint for the current stage's approach."""
        target = self.temperatures[self.currentstage]
        temp = self.controller.currenttemp
        if self.temperaturehasbeenreached or temp <= 0:
            return
        if target <= temp + 0.25 or target - temp > self.rampzone or self.fastapproach[self.currentstage]:
            # Approaching from above (no active cooling, ramp is pointless),
            # still far below, or this stage opted out of the ramp entirely
            # (fast_approach): direct setpoint, so the aggressive profile
            # gives full power all the way to target instead of being capped
            # to trickle in behind the measured temperature.
            self.rampstart = None
            self.controller.setSP(target)
            return
        if self.rampstart is None:
            self.rampstart = self.now()
            self.rampstartsp = temp
            print("Approach ramp started at", temp, "C, ramping to", target,
                  "C at", round(self.ramprate * 60, 2), "C/min")
        ramped = self.rampstartsp + (self.now() - self.rampstart) * self.ramprate
        # Never lead the measured temperature by much: if the ramp outruns the
        # water, the element re-charges with stored heat during the final
        # climb and the overshoot comes back. The allowed lead shrinks as the
        # target nears (soft landing), and always stays below the firmware's
        # 4.5 C aggressive-profile switch.
        lead = min(2.0, max(0.5, 0.4 * (target - temp)))
        ramped = min(ramped, temp + lead)
        if ramped >= target:
            sp = target
        else:
            # Quantize so we do not spam the MCU with tiny SetSP changes.
            sp = round(ramped * 4) / 4.0
        self.controller.setSP(sp)

    def applyProgram(self):
        if not self.currentstage <= len(self.temperatures)-1:
            if not self.endannounced:
                self.endannounced = True
                print("Program ended, Set Point is :",self.controller.SP)
            return
        self.updateapproachramp()
        # The stage target counts as reached only once the ramp has handed the
        # final value to the MCU (during the ramp, SP < target and the water
        # tracking the ramp must not end the stage early).
        rampdone = self.controller.SP == self.temperatures[self.currentstage]
        # Temperature is reached: start the hold timer. currentstage is NOT
        # advanced here - it always refers to the stage currently being
        # approached/held, advancing only once that stage's hold duration has
        # actually elapsed (below). Advancing it here instead (as this used
        # to) pushes the index out of range the instant the LAST stage's
        # target is reached, making the program report "ended" - and skip
        # that stage's completion chime - without ever honoring its hold
        # duration.
        if rampdone and self.controller.currenttemp  + self.temperaturetolerance>= self.controller.SP and self.controller.currenttemp - self.temperaturetolerance <= self.controller.SP and not self.temperaturehasbeenreached:
            self.temperaturehasbeenreached = True
            self.startnextsteptime = self.now() + self.waitsecondsaftertempreached[self.currentstage]
            print("Temperature reached, next stage in : ", self.waitsecondsaftertempreached[self.currentstage],"seconds")
            if self.effects[self.currentstage] == "chime":
                self.playchime(chime.info)

        # Temperature is reached and wait time ended: advance to the next
        # stage, or finish the program if this was the last one.
        if self.temperaturehasbeenreached and self.now() >= self.startnextsteptime:
            self.temperaturehasbeenreached = False
            self.rampstart = None  # the new stage plans its own approach ramp
            finishedstage = self.currentstage
            self.currentstage += 1
            if self.effects[finishedstage] == "chime":
                self.playchime(chime.success)
            if self.currentstage <= len(self.temperatures) - 1:
                self.controller.setSP(self.temperatures[self.currentstage])
                self.controller.overridepid = self.overridepid[self.currentstage]
                if self.overridepid[self.currentstage]:
                    self.controller.setPIDOverride()
                self.applystagetunings(self.currentstage)
                print("Waiting time ended, next target temperature :",self.controller.SP)
