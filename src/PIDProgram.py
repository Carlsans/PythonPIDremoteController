import os
import time
import chime

class PIDProgram:
    """Runs a staged temperature program: for each stage, heat to the stage
    temperature, then hold it for the stage duration before moving on.

    `stages` is a list of dicts: {"temperature": C, "duration_minutes": min,
    "override_pid": bool (optional)}. When omitted, the historical default
    (39 C held for 24 h) is used.
    """
    def __init__(self,controller, stages=None, tunings=None, timesource=time.time,
                 approachtunings=(100.0, 0.0, 0.0),
                 ramprate=0.5 / 60.0, rampzone=10.0):
        self.controller = controller
        self.now = timesource
        if tunings is None:
            tunings = (20, .02, .4)  # "medium-pot-5-jars" profile
        # The firmware uses the aggressive profile while |SP - temp| >= 4.5 C
        # and the conservative one when closer. The profile the user tuned
        # goes in cons (it holds the temperature); agg gets pure P with Ki=0
        # so the integral cannot wind up during long climbs - windup there is
        # what caused massive overshoot when heating to a low setpoint like
        # 38 C, where the pot barely cools passively. (Setting both profiles
        # to the same PI, as setAllPID did, defeats this firmware feature.)
        self.controller.setconsPIDvalues(tunings[0], tunings[1], tunings[2])
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
        if target <= temp + 0.25 or target - temp > self.rampzone:
            # Approaching from above (no active cooling, ramp is pointless) or
            # still far below: direct setpoint (the aggressive profile gives
            # full power until the ramp zone).
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
        # Temperature is reached
        if rampdone and self.controller.currenttemp  + self.temperaturetolerance>= self.controller.SP and self.controller.currenttemp - self.temperaturetolerance <= self.controller.SP and not self.temperaturehasbeenreached:
            self.temperaturehasbeenreached = True
            self.startnextsteptime = self.now() + self.waitsecondsaftertempreached[self.currentstage]
            print("Temperature reached, next stage in : ", self.waitsecondsaftertempreached[self.currentstage],"seconds")
            if self.effects[self.currentstage] == "chime":
                self.playchime(chime.info)

            self.currentstage += 1

        # Temperature is reached and wait time ended
        if self.temperaturehasbeenreached and self.now() >= self.startnextsteptime and self.currentstage <= len(self.temperatures)-1:
            self.temperaturehasbeenreached = False
            self.rampstart = None  # the new stage plans its own approach ramp
            self.controller.setSP(self.temperatures[self.currentstage])
            self.controller.overridepid = self.overridepid[self.currentstage]
            if self.overridepid[self.currentstage]:
                self.controller.setPIDOverride()
            print("Waiting time ended, next target temperature :",self.controller.SP)
            if self.effects[self.currentstage] == "chime":
                self.playchime(chime.success)
