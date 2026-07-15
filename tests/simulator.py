"""Simulated pot + MCU PID, used by the automated tests.

The thermal model is a first-order process with dead time, which is a good
approximation of a pot of milk on an electric heater. The PID mimics the
Arduino PID library running on the ESP8266 (integral accumulation with
output clamping, derivative on measurement).
"""

from collections import deque


class SimulatedPot:
    """First-order-plus-dead-time thermal model. One step() = one second."""

    def __init__(self, ambient=22.0, fullpowerrise=90.0, tau=1500.0, deadtime=20.0):
        self.ambient = ambient
        self.fullpowerrise = fullpowerrise  # steady-state rise above ambient at output=255
        self.tau = tau
        self.temp = ambient
        # Dead time is modelled as a delay line on the heater command.
        self.delayline = deque([0.0] * int(deadtime), maxlen=max(1, int(deadtime)))

    def step(self, output):
        """Advance one second with heater command `output` in 0..255."""
        self.delayline.append(min(max(output, 0.0), 255.0))
        applied = self.delayline[0]
        target = self.ambient + self.fullpowerrise * applied / 255.0
        self.temp += (target - self.temp) / self.tau
        return self.temp


class TwoCapacityPot:
    """Element + water thermal model, fitted to the real pot's measured
    behaviour on 2026-07-13: at full power the element stores ~8.3 C-of-water
    of heat which keeps flowing in for ~10 min after the output cuts (that
    stored heat caused an 8.3 C overshoot at a 38 C setpoint even though the
    PID cut its output before the target).

    Units: heat is measured in 'C of water'; celem is the element capacity
    relative to the water.
    """

    def __init__(self, ambient=22.0, fullrate=0.042, celem=0.34,
                 kelem=0.00136, kloss=None, deadtime=20.0, outmax=160.0):
        from collections import deque as _deque
        self.ambient = ambient
        self.fullrate = fullrate      # C/s into the element at full output
        self.celem = celem
        self.kelem = kelem            # element->water coupling, 1/s per C
        self.kloss = kloss if kloss is not None else 1.0 / 4300.0
        self.outmax = outmax
        self.temp = ambient           # water
        self.elemtemp = ambient
        self.delayline = _deque([0.0] * int(deadtime), maxlen=max(1, int(deadtime)))

    def step(self, output):
        self.delayline.append(min(max(output, 0.0), self.outmax))
        pin = self.fullrate * self.delayline[0] / self.outmax
        flow = self.kelem * (self.elemtemp - self.temp)
        self.elemtemp += (pin - flow) / self.celem
        self.temp += flow - self.kloss * (self.temp - self.ambient)
        return self.temp


class SimulatedMCUPID:
    """Direct-acting PID like the Arduino PID library, 1 s sample time."""

    def __init__(self, outmin=0.0, outmax=255.0):
        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0
        self.setpoint = 0.0
        self.outmin = outmin
        self.outmax = outmax
        self.iterm = 0.0
        self.lastinput = None

    def settunings(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def resetintegral(self):
        """Matches the real firmware: its SetSP handler always calls
        myPID.Initialize()+Reset(), even if the value did not change."""
        self.iterm = 0.0
        self.lastinput = None

    def compute(self, measured):
        error = self.setpoint - measured
        self.iterm += self.ki * error
        self.iterm = min(max(self.iterm, self.outmin), self.outmax)
        dinput = 0.0 if self.lastinput is None else measured - self.lastinput
        self.lastinput = measured
        output = self.kp * error + self.iterm - self.kd * dinput
        return min(max(output, self.outmin), self.outmax)


class SimulatedAdaptiveMCUPID(SimulatedMCUPID):
    """Mimics the real firmware (PIDController.h): aggressive tunings while
    |SP - temp| >= 4.5 C, conservative when closer. Like the Arduino PID
    library, switching tunings does NOT reset the integral."""

    GAPTHRESHOLD = 4.5

    def __init__(self, outmin=0.0, outmax=160.0):
        super().__init__(outmin=outmin, outmax=outmax)
        self.agg = (0.0, 0.0, 0.0)
        self.cons = (0.0, 0.0, 0.0)

    def settuningsagg(self, kp, ki, kd):
        self.agg = (kp, ki, kd)

    def settuningscons(self, kp, ki, kd):
        self.cons = (kp, ki, kd)

    def compute(self, measured):
        gap = abs(self.setpoint - measured)
        which = self.cons if gap < self.GAPTHRESHOLD else self.agg
        self.settunings(which[0], which[1], which[2])
        return super().compute(measured)


class FakeController:
    """Stands in for YogourtFermenter when driving RelayAutotune directly."""

    def __init__(self, pid):
        self.pid = pid
        self.SP = 0
        self.currentSP = 0
        self.currenttemp = 0.0
        self.overridepid = None
        self.pidcalls = []
        self.overridecalls = 0
        self.sentmessages = []

    def setSP(self, sp):
        self.SP = sp
        self.currentSP = sp
        self.pid.setpoint = sp

    def sendMessage(self, message):
        """Raw send, matching YogourtFermenter.sendMessage() - unconditional,
        unlike setSP() which the real code only sends if the value changed.
        Used to test PIDOptimizer's integral-reset-via-resend safety trick:
        the real firmware's SetSP handler always resets the integral, even
        for an unchanged value (see OTAInputOutput.h)."""
        self.sentmessages.append(message)
        if message.startswith("SetSP("):
            value = float(message[len("SetSP("):-1])
            self.SP = value
            self.currentSP = value
            self.pid.setpoint = value
            if hasattr(self.pid, 'resetintegral'):
                self.pid.resetintegral()

    def setAllPID(self, kp, ki, kd):
        self.pidcalls.append((kp, ki, kd))
        if isinstance(self.pid, SimulatedAdaptiveMCUPID):
            self.pid.settuningsagg(kp, ki, kd)
            self.pid.settuningscons(kp, ki, kd)
        else:
            self.pid.settunings(kp, ki, kd)

    def setconsPIDvalues(self, kp, ki, kd):
        self.pidcalls.append(("cons", kp, ki, kd))
        if isinstance(self.pid, SimulatedAdaptiveMCUPID):
            self.pid.settuningscons(kp, ki, kd)
        else:
            self.pid.settunings(kp, ki, kd)

    def setaggPIDvalues(self, kp, ki, kd):
        self.pidcalls.append(("agg", kp, ki, kd))
        if isinstance(self.pid, SimulatedAdaptiveMCUPID):
            self.pid.settuningsagg(kp, ki, kd)

    def setPIDOverride(self):
        self.overridecalls += 1
