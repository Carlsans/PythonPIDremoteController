import datetime
import errno
import fcntl
import math
import os
import socket
import tempfile
import time

import numpy as np


from matplotlib import pyplot as plt, style
import matplotlib.dates as md

from src.PIDProgram import PIDProgram
from src.RelayAutotune import RelayAutotune
from src.NetworkConfiguration import NetworkConfiguration


class SingleInstanceError(Exception):
    pass


def acquireportlock(port):
    """Exclusive, OS-level lock keyed by the listen port.

    Two YogourtFermenter processes bound to the same UDP port (e.g. a
    leftover process from a previous run that looked frozen, plus a freshly
    started one) do NOT error - the kernel happily delivers each incoming
    packet to whichever socket it picks, so both processes silently run
    "successfully" while randomly losing packets to each other and each
    sending their own (possibly stale/conflicting) commands to the real ESP.
    This was confirmed happening for hours at a stretch via
    graph_diagnostics.log. Refusing to start a second instance for the same
    port makes that impossible instead of relying on the user noticing.

    flock() is tied to the file descriptor: even a killed (SIGKILL) process
    releases it automatically when the OS closes its file descriptors, so
    there is no stale-lock cleanup to worry about.
    """
    lockpath = os.path.join(tempfile.gettempdir(), "yogurtfermenter_port_" + str(port) + ".lock")
    lockfile = open(lockpath, 'w')
    try:
        fcntl.flock(lockfile, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError as e:
        if e.errno in (errno.EACCES, errno.EAGAIN):
            lockfile.close()
            raise SingleInstanceError(
                "Another YogourtFermenter process is already using port " + str(port)
                + ". Running two instances against the same ESP makes them silently race "
                "for incoming packets and send it conflicting commands. Close the other "
                "instance (check `ps aux | grep yogurt`) before starting a new one.")
        raise
    lockfile.write(str(os.getpid()) + "\n")
    lockfile.flush()
    return lockfile  # kept as self.portlock so the lock lives as long as the process does


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
        self.ontickfailures = 0
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
        # Proactively recreate the graph window this often (0 disables it).
        # See recreategraph() for why this exists. Off by default: recreating
        # raises/refocuses the window, which is disruptive on its own, and no
        # freeze has actually been observed yet - use the GUI's "Refresh
        # graph" button (or call recreategraph() directly) if one ever is.
        self.graphrefreshseconds = float(os.environ.get('YOGURT_GRAPH_REFRESH_SECONDS', 0))
        self.graphrecreatecount = 0
        # Set by requestgraphrefresh() (the GUI's "Refresh graph" button) and
        # serviced from animate(), never called directly from the button's
        # callback - see requestgraphrefresh() for why.
        self.graphrefreshrequested = False
        # Diagnostic log for the graph-freeze investigation: timing of every
        # redraw call plus periodic heartbeats, so a run that visually froze
        # can be correlated afterwards with what the process actually saw.
        # Empty string disables it.
        self.diaglogpath = os.environ.get('YOGURT_GRAPH_DIAG_LOG',
                                          os.getcwd() + "/graph_diagnostics.log")
        self.diagheartbeatseconds = float(os.environ.get('YOGURT_GRAPH_DIAG_HEARTBEAT_SECONDS', 5 * 60))
        self.diagslowredrawms = float(os.environ.get('YOGURT_GRAPH_DIAG_SLOW_MS', 250))
        self.diaglastheartbeat = 0.0
        self.diagredrawcount = 0
        self.diagmaxredrawms = 0.0
        self.setPidvalues()
        self.networkconf = NetworkConfiguration()
        self.portlock = acquireportlock(self.networkconf.listen_port)
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

    def safecall(self, label, fn, *args):
        """Call fn(*args), logging and swallowing any exception instead of
        letting it kill the whole control loop (and with it the whole
        fermentation run). Returns True on success, False if fn raised.

        applyProgram(), checkconnection(), relayautotune.update() and
        ontick() used to be called unprotected here - any bug in any of them
        (a stage-list edge case, a Tk/matplotlib hiccup...) would silently
        take down the entire process with no trace of why.
        """
        try:
            fn(*args)
            return True
        except Exception as e:
            print(label, "raised an exception (ignored):", repr(e))
            self.diaglog(label.upper().replace(".", "_").replace(" ", "_") + "_EXCEPTION " + repr(e))
            return False

    def diaglog(self, message):
        if not self.diaglogpath:
            return
        try:
            with open(self.diaglogpath, 'a') as f:
                f.write(datetime.datetime.now().isoformat(sep=' ', timespec='milliseconds')
                        + " | " + message + "\n")
        except OSError:
            pass  # diagnostics must never be able to break the fermentation run

    def graphbackendinfo(self):
        import matplotlib
        backend = matplotlib.get_backend()
        qpa = os.environ.get('QT_QPA_PLATFORM', '')
        if not qpa:
            try:
                from matplotlib.backends.qt_compat import QtWidgets
                app = QtWidgets.QApplication.instance()
                if app is not None:
                    qpa = app.platformName()
            except Exception:
                qpa = ''
        return backend, qpa or 'n/a'

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
        self.lastgraphrecreate = time.time()
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
        backend, qpa = self.graphbackendinfo()
        self.diaglog("GRAPH_CREATED backend=" + backend + " qpa=" + qpa
                     + " refresh_s=" + str(self.graphrefreshseconds)
                     + " recreate_count=" + str(self.graphrecreatecount))

    def recreategraph(self):
        """Close and reopen the graph window.

        Some window managers/compositors can stop delivering redraw frames
        to a long-lived window without the process ever seeing an error (no
        crash, no exception - the control loop keeps running fine while the
        graph just silently stops updating on screen). There is no reliable
        way to detect that from inside matplotlib, so instead the window can
        be proactively recreated (see `graphrefreshseconds`), bounding how
        long the graph can stay frozen instead of the rest of a
        multi-hour/multi-day fermentation run.

        Only ever called from animate() (see requestgraphrefresh() for why).
        """
        self.diaglog("RECREATE_SCHEDULED age_s=" + str(round(time.time() - self.lastgraphrecreate, 1)))
        try:
            plt.close(self.fig)
        except Exception as e:
            print("Could not close stale graph (ignored):", e)
        self.graphrecreatecount += 1
        self.creategraph()

    def requestgraphrefresh(self):
        """Ask for the graph window to be closed and reopened (e.g. from the
        GUI's "Refresh graph" button).

        This does NOT call recreategraph() directly. The button's click
        handler runs nested deep inside the GUI's own Tk event dispatch
        (listeningloop() -> ontick() -> root.update() -> [button callback]),
        several stack frames removed from listeningloop()'s own top level -
        unlike the automatic refresh, which animate() only ever triggers
        from that top level. A real run showed a process go silent (no
        crash, just stopped producing any further output at all) within
        seconds of a manual refresh click, while an isolated repro of the
        same click could not reproduce a hang - consistent with a rare,
        context-dependent GUI/Tk reentrancy issue rather than something in
        recreategraph() itself. Setting a flag serviced from animate() makes
        the manual and automatic paths run through the exact same, simpler
        call context either way.
        """
        self.graphrefreshrequested = True

    def animate(self):

        if self.graphrefreshrequested or (
                self.graphrefreshseconds > 0 and hasattr(self, 'lastgraphrecreate')
                and time.time() - self.lastgraphrecreate > self.graphrefreshseconds):
            self.graphrefreshrequested = False
            self.recreategraph()

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
            # draw_idle() explicitly requests a redraw (flush_events() alone
            # only polls already-queued window-system events, it does not
            # request one). plt.pause() then gives the GUI toolkit's event
            # loop actual time to run, instead of a single non-blocking poll -
            # this is the pattern matplotlib's own docs recommend for
            # updating a plot from a custom loop (as opposed to FuncAnimation)
            # and is required for some window managers/compositors to
            # actually service the redraw.
            drawstart = time.monotonic()
            self.fig.canvas.draw_idle()
            plt.pause(0.001)
            drawms = (time.monotonic() - drawstart) * 1000
            self.diagredrawcount += 1
            self.diagmaxredrawms = max(self.diagmaxredrawms, drawms)
            if drawms > self.diagslowredrawms:
                # A slow call here means the draw/event-loop call itself
                # hung inside Python - as opposed to the window silently not
                # repainting on screen while Python sees nothing wrong.
                # Distinguishing those two is the whole point of this log.
                self.diaglog("SLOW_REDRAW dt_ms=" + str(round(drawms, 1)) + " n_points=" + str(n))
            self.diagheartbeat()
        except Exception as e:
            # Never let a drawing problem (e.g. window closed mid-draw) take
            # down the control loop.
            print("Plotting error (ignored):", e)
            self.diaglog("ANIMATE_EXCEPTION " + repr(e))

    def diagheartbeat(self):
        now = time.time()
        if now - self.diaglastheartbeat < self.diagheartbeatseconds:
            return
        self.diaglastheartbeat = now
        self.diaglog("HEARTBEAT temp=" + str(self.currenttemp) + " sp=" + str(self.currentSP)
                     + " cv=" + str(self.currentCV) + " redraws=" + str(self.diagredrawcount)
                     + " max_redraw_ms=" + str(round(self.diagmaxredrawms, 1))
                     + " points=" + str(len(self.tempbysec))
                     + " fig_stale=" + str(self.fig.stale)
                     + " recreate_count=" + str(self.graphrecreatecount))
        self.diagredrawcount = 0
        self.diagmaxredrawms = 0.0

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
                self.safecall("checkconnection", self.checkconnection)
                if self.mode == 'relayautotune':
                    self.safecall("relayautotune.update", self.relayautotune.update, self.currenttemp)
                count += 1
                if count > 5:
                    count = 0
                    self.savetempbysectocsv()
            if self.mode == 'pidprogram':
                self.safecall("pidprogram.applyProgram", self.pidprogram.applyProgram)
            if self.ontick is not None:
                if self.safecall("ontick", self.ontick):
                    self.ontickfailures = 0
                else:
                    self.ontickfailures += 1
                    if self.ontickfailures >= 10:
                        print("ontick() failed 10 times in a row - assuming the GUI is gone, stopping.")
                        self.diaglog("ONTICK_GIVING_UP after 10 consecutive failures")
                        self.stoprequested = True

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
        try:
            fcntl.flock(self.portlock, fcntl.LOCK_UN)
            self.portlock.close()
        except OSError:
            pass
        try:
            # Without this, a second YogourtFermenter instance created later
            # in the same process (e.g. the GUI's "Autotune here" flow: stop
            # -> autotune -> stop -> resume, three instances in a row) finds
            # plt.get_fignums() still non-empty, so creategraph() silently
            # no-ops and self.ax1 etc. never get set on the new instance -
            # every subsequent animate() call then raises AttributeError
            # (caught, so it doesn't crash the run, but the graph never
            # reappears for the rest of that instance's life).
            plt.close(self.fig)
        except Exception:
            pass
        print("Listening loop stopped.")


if __name__ == '__main__':
    yogourtfermenter = YogourtFermenter()
