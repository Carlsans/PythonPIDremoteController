import datetime
import math
import os
import socket
import time

import numpy as np


from matplotlib import pyplot as plt, style
import matplotlib.dates as md

from src.PIDProgram import PIDProgram
from src.RelayAutotune import RelayAutotune
from src.NetworkConfiguration import NetworkConfiguration
class YogourtFermenter():
    def __init__(self, mode=None, stages=None, tunings=None,
                 autotunetarget=None, onautotunedone=None,
                 ontick=None, autorun=True):
        modes = ['pidprogram','relayautotune']
        if mode is None:
            mode = os.environ.get('YOGURT_MODE', modes[0])
        self.mode = mode
        if self.mode not in modes:
            raise ValueError("Unknown mode '" + self.mode + "'. Valid modes: " + str(modes))
        # Called on every loop iteration (several times per second); the GUI
        # uses it to stay responsive while the loop runs.
        self.ontick = ontick
        self.stoprequested = False
        self.tempbysec = []
        self.CV = []
        self.SPlist = []
        self.PIDTermslist = []
        self.errorlist = []
        self.SP = 40
        self.overridepid = None
        self.currentSP = self.SP
        self.currentCV = 0
        self.currenttemp = 0
        self.lasttemp = 0
        self.sametempcount = 0
        self.currentoutput = 0
        self.currentPIDTerms = [0,0,0]
        self.currentPIDSettings = [0,0,0]
        self.starttime = datetime.datetime.now()
        self.trimmedseconds = 0
        # If no packet arrives for this long, the connection is considered
        # dead and the UDP socket is recreated.
        self.connectionlostseconds = float(os.environ.get('YOGURT_CONNECTION_TIMEOUT', 30))
        self.lastpackettime = time.time()
        self.setPidvalues()
        self.networkconf = NetworkConfiguration()
        self.createsocket()
        self.creategraph()
        if self.mode == 'pidprogram':
            self.pidprogram = PIDProgram(self, stages=stages, tunings=tunings)
        if self.mode == 'relayautotune':
            if autotunetarget is None:
                autotunetarget = float(os.environ.get('YOGURT_AUTOTUNE_TARGET', self.SP))
            # Bound the relay's "heater on" setpoint a bit above the target so
            # high targets like 82 C work but nothing can approach boiling.
            maxsafe = min(95.0, autotunetarget + 12)
            self.relayautotune = RelayAutotune(self, targettemp=autotunetarget,
                                               maxsafetemp=maxsafe,
                                               oncomplete=onautotunedone)

        if autorun:
            self.listeningloop()

    def createsocket(self):
        old_sock = getattr(self, 'sock', None)
        if old_sock is not None:
            try:
                old_sock.close()
            except OSError as e:
                print(f"Warning: error closing old socket: {e}")
        # Create a UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # A timeout keeps the loop (and the graph/GUI) alive when the MCU
        # stops sending: recvfrom() raises socket.timeout instead of blocking.
        self.sock.settimeout(0.2)

        # Set the IP address and port to listen on
        server_address = ('', self.networkconf.listen_port)
        self.sock.bind(server_address)
        print('Listening on {}:{}'.format(*server_address))

    def setPidvalues(self):
        # Initial values matching the firmware defaults; the program or the
        # autotuner replaces them right after startup.
        multiplier = 2.56
        self.aggKp = 3.27 * multiplier
        self.aggKi = 0.0008 * multiplier
        self.aggKd = 646.15 * multiplier
        self.consKp = 3 * multiplier
        self.consKi = 0.001 * multiplier
        self.consKd = 6.282 * multiplier


    def creategraph(self):
        if plt.get_fignums():
            print("Graphic already opened")
            return
        style.use('fivethirtyeight')

        self.fig , ((self.ax1, self.ax2),(self.ax3, self.ax4)) = plt.subplots(2, 2, figsize=(10, 5))
        self.fig.subplots_adjust(hspace=0.6, wspace=0.3)
        plt.ion()

        xfmt = md.DateFormatter('%Hh%Mm')
        xfmt2 = md.DateFormatter('%Ss')
        self.ax1.xaxis.set_major_formatter(xfmt)
        self.ax2.xaxis.set_major_formatter(xfmt)
        self.ax3.xaxis.set_major_formatter(xfmt2)
        self.ax4.xaxis.set_major_formatter(xfmt)
        self.ax2.set_title("P="+str(self.currentPIDSettings[0])+"I="+str(self.currentPIDSettings[1])+"D="+str(self.currentPIDSettings[2]))
        self.ax1.plot([], [],label="Temp")
        self.ax1.plot([], [],label="PID")
        self.ax2.plot([], [],label="P")
        self.ax2.plot([], [],label="I")
        self.ax2.plot([], [],label="D")
        self.ax3.plot([], [],label="Latest Temp")
        self.ax4.plot([], [], label="Temperature last change")
        self.ax1.legend()
        self.ax2.legend()
        self.fig.show()  # show the window (figure will be in foreground, but the user may move it to background)


    def animate(self):

        if not plt.get_fignums():
            print("No graph")
            return
        if self.tempbysec == []:
            return
        try:
            x = []
            for i in range(len(self.tempbysec)):
                x.append(self.starttime + datetime.timedelta(seconds=i - 1 + self.trimmedseconds))

            # The per-second lists are appended together, but guard against a
            # transient mismatch so a plotting hiccup can never kill the loop.
            n = min(len(x), len(self.tempbysec), len(self.CV), len(self.PIDTermslist))
            x = x[:n]

            self.ax1.lines[0].set_data(x, self.tempbysec[:n])  # Temperature
            self.ax1.lines[1].set_data(x, self.CV[:n])  # PID PWM
            self.ax1.lines[0].set_zorder(2)
            self.ax1.set_title("MCU target="+str(self.currentSP))


            self.ax2.lines[0].set_data(x, [i[0] for i in self.PIDTermslist[:n]])
            self.ax2.lines[1].set_data(x, [i[1] for i in self.PIDTermslist[:n]])
            self.ax2.lines[2].set_data(x, [i[2] for i in self.PIDTermslist[:n]])
            self.ax2.set_title("P="+str(self.currentPIDSettings[0])+"I="+str(self.currentPIDSettings[1])+"D="+str(self.currentPIDSettings[2]))

            if n>100:
                self.ax3.lines[0].set_data(x[-100:], self.tempbysec[n-100:n])  # Temperature
            else:
                self.ax3.lines[0].set_data(x, self.tempbysec[:n])  # Temperature
            self.ax3.set_title("Current temp=" + str(self.currenttemp))
            self.ax4.lines[0].set_data([i[0] for i in self.errorlist], [i[1] for i in self.errorlist])

            self.ax1.set_title("MCU target=" + str(self.currentSP))

            self.ax1.relim()  # recompute the data limits
            self.ax1.autoscale_view()  # automatic axis scaling
            self.ax2.relim()  # recompute the data limits
            self.ax2.autoscale_view()  # automatic axis scaling
            self.ax3.relim()  # recompute the data limits
            self.ax3.autoscale_view()  # automatic axis scaling
            self.ax4.relim()  # recompute the data limits
            self.ax4.autoscale_view()  # automatic axis scaling
            self.fig.canvas.flush_events()
        except Exception as e:
            # Never let a drawing problem (e.g. window closed mid-draw) take
            # down the control loop.
            print("Plotting error (ignored):", e)

    def appendnewtemperatureevent(self,message):
        message = message.replace("Sensor temp: ","")

        self.currenttemp = float(message)
        if self.currenttemp == self.lasttemp:
            self.sametempcount +=1
        else:
            self.sametempcount = 0
            self.lasttemp = self.currenttemp
        resettemptime = 120 * 3
        if self.sametempcount > resettemptime:
            # Sensor value frozen: the MCU may be stuck, reset our socket.
            self.createsocket()
            print("Temperature stable for more that "+str(resettemptime)+"s. Resetting UDP connection.")
            self.sametempcount = 0
        self.errorlist.append([datetime.datetime.now(), float(self.sametempcount)])


    def line_prepender(self,filename, line):
        with open(filename, 'r+') as f:
            content = f.read()
            f.seek(0, 0)
            f.write(line.rstrip('\r\n') + '\n' + content)
    def savetempbysectocsv(self):
        try:
            array = np.zeros((3,len(self.tempbysec)))
            array[0] = self.tempbysec
            array[1] = self.CV
            array[2] = self.SPlist
            filepath = os.getcwd()+"/yogurtpid2.csv"
            np.savetxt(filepath,
                       array.T,
                       delimiter=";",
                       fmt='% s')
            self.line_prepender(filepath,"PV;CV;SP")
        except (OSError, ValueError) as e:
            print("Could not save CSV (ignored):", e)
    def sendMessage(self,message):
        try:
            self.sock.sendto(message.encode(), (self.networkconf.esp8266_ip, self.networkconf.esp8266_port))
        except OSError as e:
            print("Could not send '" + message + "':", e)
    def setAllPID(self,Kp,Ki,Kd):
        print("Setting new pid params for all",Kp,Ki,Kd)
        self.aggKp = Kp
        self.consKp = Kp
        self.aggKi = Ki
        self.consKi = Ki
        self.aggKd = Kd
        self.consKd = Kd
        self.setaggPID()
        self.setconsPID()
    def setconsPIDvalues(self, Kp, Ki, Kd):
        print("Setting cons pid params", Kp, Ki, Kd)
        self.consKp = Kp
        self.consKi = Ki
        self.consKd = Kd
        self.setconsPID()
    def setaggPIDvalues(self, Kp, Ki, Kd):
        print("Setting agg pid params", Kp, Ki, Kd)
        self.aggKp = Kp
        self.aggKi = Ki
        self.aggKd = Kd
        self.setaggPID()
    def setaggPID(self):
        self.sendMessage("SetTuningsagg("+str(self.aggKp)+","+str(self.aggKi)+","+str(self.aggKd) + ")")
    def setconsPID(self):
        self.sendMessage("SetTuningscons("+str(self.consKp)+","+str(self.consKi)+","+str(self.consKd) + ")")
    def setSP(self, SP):
        self.SP = SP
        if self.currentSP == SP:
            return  # MCU already there; nothing to send
        print("Setting new temperature target at :",SP)
        self.sendMessage("SetSP(" + str(self.SP) + ")")
    def setPIDOverride(self):
        if self.overridepid:
            print("Overriding PID")
            self.sendMessage("OverridePID")
        else:
            print("Stop PID Override and perform pid test")
            self.sendMessage("EnablePIDandTest")
    def cleanPlotlists(self):
        maxlen = 60*60*20
        if len(self.tempbysec) > maxlen:
            self.tempbysec = self.tempbysec[60*60:]
            self.CV = self.CV[60*60:]
            self.SPlist = self.SPlist[60*60:]
            self.PIDTermslist = self.PIDTermslist[60*60:]
            self.trimmedseconds += 60*60
        # errorlist grows with every packet, not once per second: trim it too
        # or matplotlib slows down and eventually freezes on long runs.
        if len(self.errorlist) > maxlen:
            self.errorlist = self.errorlist[60*60:]
    def checkconnection(self):
        if time.time() - self.lastpackettime > self.connectionlostseconds:
            print("No packet received for " + str(self.connectionlostseconds) + "s. Resetting UDP connection.")
            self.createsocket()
            # Ask the MCU for a resync: if it is reachable again it will echo
            # its SetPoint and the normal resync logic takes over.
            self.sendMessage("SetSP(" + str(self.SP) + ")")
            self.lastpackettime = time.time()
    def parsefloatlist(self, message, prefix):
        return [float(part) for part in message.replace(prefix, "").split(',')]
    def handlemessage(self, message):
        if "SetPoint:" in message:
            self.currentSP = float(message.replace("SetPoint:",""))
            if not self.currentSP == self.SP:
                self.setSP(self.SP)
                print("self.currentSP,self.SP",self.currentSP,self.SP)
                print("SP was not set !! Resetting it ....")
            return
        if "Override PID:" in message:
            message = message.replace("Override PID: ","")
            if "0" in message and self.overridepid or "1" in message and not self.overridepid:
                self.setPIDOverride()
                print("PID Override not set. Setting PID Overide to ",self.overridepid )
            return
        if "Sensor temp:" in message:
            self.appendnewtemperatureevent(message)
            return
        if "Output:" in message:
            message = message.replace("Output:","")
            if "nan" in message:
                self.currentCV = 0
                self.currentoutput = 0
                return
            self.currentoutput = float(message)  # raw MCU output (0..255 scale)
            self.currentCV = int((self.currentoutput/255)*100)
            return
        if "PIDsettings(Kp,Ki,Kd):" in message:
            answers = self.parsefloatlist(message, "PIDsettings(Kp,Ki,Kd):")
            if len(answers) == 3:
                self.currentPIDSettings = answers
            return
        if "PIDsettingscons(Kp,Ki,Kd):" in message:
            answers = self.parsefloatlist(message, "PIDsettingscons(Kp,Ki,Kd):")
            if len(answers) == 3 and not (math.isclose(answers[0], self.consKp, abs_tol=0.00001) and math.isclose(answers[1], self.consKi, abs_tol=0.00001) and math.isclose(answers[2], self.consKd, abs_tol=0.00001)):
                self.setconsPID()
            return
        if "PIDsettingsagg(Kp,Ki,Kd):" in message:
            answers = self.parsefloatlist(message, "PIDsettingsagg(Kp,Ki,Kd):")
            if len(answers) == 3 and not (math.isclose(answers[0], self.aggKp, abs_tol=0.00001) and math.isclose(answers[1], self.aggKi, abs_tol=0.00001) and math.isclose(answers[2], self.aggKd, abs_tol=0.00001)):
                self.setaggPID()
            return
        if "PID Terms: " in message:
            answers = self.parsefloatlist(message, "PID Terms: ")
            if len(answers) == 3:
                self.currentPIDTerms = answers
            return
    def listeningloop(self):
        lastsec = int(time.time())
        print(lastsec)
        count = 0
        while not self.stoprequested:
            if time.time() > lastsec + 1:
                lastsec = int(time.time())
                self.tempbysec.append(self.currenttemp)
                self.CV.append(self.currentCV)
                self.SPlist.append(self.currentSP)
                self.PIDTermslist.append(self.currentPIDTerms)
                self.cleanPlotlists()
                self.animate()
                self.checkconnection()
                if self.mode == 'relayautotune':
                    self.relayautotune.update(self.currenttemp)
                count += 1
                if count > 5:
                    count = 0
                    self.savetempbysectocsv()
            if self.mode == 'pidprogram':
                self.pidprogram.applyProgram()
            if self.ontick is not None:
                self.ontick()

            try:
                data, address = self.sock.recvfrom(4096)
            except socket.timeout:
                # No data this second: loop again so the graph stays alive.
                continue
            except OSError as e:
                print("Socket error:", e)
                self.createsocket()
                continue
            self.lastpackettime = time.time()
            try:
                message = data.decode()
            except UnicodeDecodeError:
                print("Ignoring undecodable packet:", data[:80])
                continue
            print(message)
            try:
                self.handlemessage(message)
            except (ValueError, IndexError) as e:
                # A truncated or corrupted UDP packet must never crash the
                # program (this used to kill the graph).
                print("Ignoring malformed message", repr(message), ":", e)
        # Loop stopped (GUI stop button or shutdown): release the port so a
        # new run can bind it.
        try:
            self.sock.close()
        except OSError:
            pass
        print("Listening loop stopped.")


if __name__ == '__main__':
    yogourtfermenter = YogourtFermenter()
