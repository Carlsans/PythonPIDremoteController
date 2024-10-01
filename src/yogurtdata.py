import datetime
import math
import os
import socket
import time

import numpy as np


from matplotlib import pyplot as plt, style
import matplotlib.dates as md

from src.PIDProgram import PIDProgram
from src.OptimizePID import OptimizePID
from src.NetworkConfiguration import NetworkConfiguration
class YogourtFermenter():
    def __init__(self):
        modes = ['optimizepid','pidprogram']
        self.mode = modes[1]
        self.temperatureevents = []
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
        self.currentoutput = 0
        self.currentPIDTerms = [0,0,0]
        self.currentPIDSettings = [0,0,0]
        self.currenterror = 0
        self.starttime = datetime.datetime.now()
        self.setPidvalues()
        # Create a UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.networkconf = NetworkConfiguration()

        # Set the IP address and port to listen on
        server_address = ('', self.networkconf.esp8266_port)

        self.sock.bind(server_address)
        print('Listening on {}:{}'.format(*server_address))
        self.creategraph()
        if self.mode == 'pidprogram':
            self.pidprogram = PIDProgram(self)
            self.setaggPID()
            self.setconsPID()
            #mul=1.2
            #self.setAllPID(25.517766532192542*mul, 8.562967413771423*mul, 4.673292278251236*mul)
        if self.mode == 'optimizepid':
            self.optimizepid = OptimizePID(self)

        self.listeningloop()


    def setPidvalues(self):
        #float aggKp = 3.27, aggKi = 0.0008, aggKd = 646.15;
        multiplier = 2.56
        self.aggKp = 3.27 * multiplier
        self.aggKi = 0.0008 * multiplier
        self.aggKd = 646.15 * multiplier
        # consKp = 1.91, consKi = 0.0004, consKd = 6.282;
        self.consKp = 3 * multiplier
        #self.consKi = 0.0004 * multiplier
        self.consKi = 0.001 * multiplier # 0.035
        self.consKd = 6.282 * multiplier


    def creategraph(self):
        if plt.get_fignums():
            print("Graphic already opened")
            return
        self.graphstarttime = time.time()
        style.use('fivethirtyeight')

        #self.fig = plt.figure()
        self.fig , ((self.ax1, self.ax2),(self.ax3, self.ax4)) = plt.subplots(2, 2, figsize=(10, 5))
        self.fig.subplots_adjust(hspace=0.6, wspace=0.3)
        #self.fig.canvas.mpl_connect('close_event', self.pltwindowclosed)
        #self.ax1 = self.fig.add_subplot(1, 1, 1)
        #self.ax2 = self.fig.add_subplot(1, 1, 1)
        plt.ion()
        # plt.show()

        xfmt = md.DateFormatter('%Hh%Mm')
        xfmt2 = md.DateFormatter('%Ss')
        self.ax1.xaxis.set_major_formatter(xfmt)
        self.ax2.xaxis.set_major_formatter(xfmt)
        self.ax3.xaxis.set_major_formatter(xfmt2)
        self.ax2.set_title("P="+str(self.currentPIDSettings[0])+"I="+str(self.currentPIDSettings[1])+"D="+str(self.currentPIDSettings[2]))
        self.line1 = self.ax1.plot([], [],label="Temp")  # add an empty line to the plot
        self.line2 = self.ax1.plot([], [],label="PID")
        self.line3 = self.ax2.plot([], [],label="P")
        self.line4 = self.ax2.plot([], [],label="I")
        self.line5 = self.ax2.plot([], [],label="D")
        self.line6 = self.ax3.plot([], [],label="Latest Temp")
        self.ax1.legend()
        self.ax2.legend()
        self.fig.show()  # show the window (figure will be in foreground, but the user may move it to background)


    def animate(self):

        if not plt.get_fignums():
            print("No graph")
            return
        if self.temperatureevents == []:
            return
        # self.ax1.clear()
        # self.ax1.plot([i[0] for i in self.temperatureevents], [i[1] for i in self.temperatureevents])
        firsttimeevent = self.temperatureevents[0][0]
        x, y = [datetime.datetime.fromtimestamp((i[0] - firsttimeevent).total_seconds()) for i in self.temperatureevents], [
            i[1] for i in self.temperatureevents]
        #print("firsttimeevent", firsttimeevent)
        #print(self.temperatureevents, "self.temperatureevents")
        #print(x, y)
        #self.line1.set_data(x, y)
        x = []
        for i in range(len(self.tempbysec)):
            x.append(self.starttime + datetime.timedelta(seconds=i-1))

        self.ax1.lines[0].set_data(x, self.tempbysec)  # Temperature
        self.ax1.lines[1].set_data(x, self.CV)  # PID PWM
        self.ax1.lines[0].set_zorder(2)
        self.ax1.set_title("MCU target="+str(self.currentSP))


        self.ax2.lines[0].set_data(x, [i[0] for i in self.PIDTermslist])
        self.ax2.lines[1].set_data(x, [i[1] for i in self.PIDTermslist])
        self.ax2.lines[2].set_data(x, [i[2] for i in self.PIDTermslist])
        self.ax2.set_title("P="+str(self.currentPIDSettings[0])+"I="+str(self.currentPIDSettings[1])+"D="+str(self.currentPIDSettings[2]))

        if len(x)>100:
            self.ax3.lines[0].set_data(x[-100:], self.tempbysec[-100:])  # Temperature
        else:
            self.ax3.lines[0].set_data(x, self.tempbysec)  # Temperature
        self.ax3.set_title("Current temp=" + str(self.currenttemp))

        self.ax1.set_title("MCU target=" + str(self.currentSP))

        self.ax1.relim()  # recompute the data limits
        self.ax1.autoscale_view()  # automatic axis scaling
        self.ax2.relim()  # recompute the data limits
        self.ax2.autoscale_view()  # automatic axis scaling
        self.ax3.relim()  # recompute the data limits
        self.ax3.autoscale_view()  # automatic axis scaling
        self.fig.canvas.flush_events()

    def appendnewtemperatureevent(self,message):
        message = message.replace("Sensor temp: ","")

        # temperature = float(data['StatusSNS']['MAX6675']['Temperature'])
        # mesuretime = data['StatusSNS']['Time']
        # print(mesuretime)
        # mesuretime = parser.parse(mesuretime)
        # print(mesuretime)
        self.currenttemp = float(message)
        self.temperatureevents.append([datetime.datetime.now(), float(message)])
        #print(self.temperatureevents)


    def line_prepender(self,filename, line):
        with open(filename, 'r+') as f:
            content = f.read()
            f.seek(0, 0)
            f.write(line.rstrip('\r\n') + '\n' + content)
    def savetempbysectocsv(self):
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

        # tempbysec_dict = {'PV': self.tempbysec, 'CV': self.CV,'SP' : [str(self.SP).replace(',','.') for i in self.CV]}
        # tempbysec_df = pd.DataFrame(tempbysec_dict)
        # tempbysec_df.to_csv(os.getcwd()+"/yogurtpid.csv")
    def sendMessage(self,message):
        self.sock.sendto(message.encode(), (self.networkconf.esp8266_ip, self.networkconf.esp8266_port))
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
    def setaggPID(self):
        self.sendMessage("SetTuningsagg("+str(self.aggKp)+","+str(self.aggKi)+","+str(self.aggKd) + ")")
    def setconsPID(self):
        self.sendMessage("SetTuningscons("+str(self.consKp)+","+str(self.consKi)+","+str(self.consKd) + ")")
    def setSP(self, SP):
        if self.currentSP == SP:
            return
        print("Setting new temperature target at :",SP)
        self.SP = SP
        self.sendMessage("SetSP(" + str(self.SP) + ")")
    def isSPsetMCU(self):
        return self.currentSP == self.SP
    def setManualPIDTuning(self):
        self.sendMessage("ExternalPIDTest")
    def setPIDOverride(self):
        if self.overridepid:
            print("Overriding PID")
            self.sendMessage("OverridePID")
        else:
            print("Stop PID Override and perform pid test")
            self.sendMessage("EnablePIDandTest")
    def cleanPlotlists(self):
        if len(self.tempbysec) > 60*60*20:
            self.tempbysec = self.tempbysec[60*60:]
            self.CV = self.CV[60*60:]
            self.SPlist = self.SPlist[60*60:]
            self.PIDTermslist = self.PIDTermslist[60*60:]
    def listeningloop(self):
        lastsec = int(time.time())
        print(lastsec)
        count = 0
        while True:
            if time.time() > lastsec + 1:
                lastsec = int(time.time())
                self.tempbysec.append(self.currenttemp)
                self.CV.append(self.currentCV)
                self.SPlist.append(self.currentSP)
                self.PIDTermslist.append(self.currentPIDTerms)
                self.cleanPlotlists()
                self.animate()
                #self.errorlist.append(self.currenterror)
                if self.mode == 'optimizepid':
                    self.optimizepid.setSetPoint(self.currentSP)
                    self.optimizepid.optimizePID(self.currenttemp)

                    #print(self.optimizepid)
                #print("self.tempbysec",self.tempbysec)
                #self.setaggPID()
                #self.setconsPID()
                #self.setSP(33)
                #self.setManualPIDTuning()
                count += 1
            if self.mode == 'pidprogram':
                self.pidprogram.applyProgram()

            data, address = self.sock.recvfrom(4096)
            message = data.decode()
            print(message)
            #print(time.time())
            if count > 5:
                count = 0
                self.savetempbysectocsv()
            if "SetPoint:" in message:
                self.currentSP = float(message.replace("SetPoint:",""))
                if not self.currentSP == self.SP:
                    self.setSP(self.SP)
                    print("self.currentSP,self.SP",self.currentSP,self.SP)
                    print("SP was not set !! Resetting it ....")
                continue
            if "Override PID:" in message:
                message = message.replace("Override PID: ","")
                if "0" in message and self.overridepid or "1" in message and not self.overridepid:
                    self.setPIDOverride()
                    print("PID Override not set. Setting PID Overide to ",self.overridepid )
                continue
            if "Sensor temp:" in message:
                self.appendnewtemperatureevent(message)
                continue
            if "Output:" in message:
                message = message.replace("Output:","")
                if "nan" in message:
                    self.currentCV = 0
                    continue
                self.currentCV = int((float(message)/255)*100)
                #print("self.currentCV",self.currentCV)
                continue
            if "PIDsettings(Kp,Ki,Kd):" in message:
                message = message.replace("PIDsettings(Kp,Ki,Kd):", "")
                currentvaluestr = ""
                answers = []
                for i in message:
                    if i == ',':
                        answers.append(float(currentvaluestr))
                        currentvaluestr = ""
                        continue
                    currentvaluestr += i
                answers.append(float(currentvaluestr))
                self.currentPIDSettings = answers
            if "PIDsettingscons(Kp,Ki,Kd):" in message:
                message = message.replace("PIDsettingscons(Kp,Ki,Kd):", "")
                currentvaluestr = ""
                answers = []
                for i in message:
                    if i == ',':
                        answers.append(float(currentvaluestr))
                        currentvaluestr = ""
                        continue
                    currentvaluestr += i
                answers.append(float(currentvaluestr))
                #print(answers)
                if not (math.isclose(answers[0], self.consKp, abs_tol=0.00001) and math.isclose(answers[1], self.consKi, abs_tol=0.00001) and math.isclose(answers[2], self.consKd, abs_tol=0.00001)):
                    self.setconsPID()
            if "PIDsettingsagg(Kp,Ki,Kd):" in message:
                message = message.replace("PIDsettingsagg(Kp,Ki,Kd):", "")
                currentvaluestr = ""
                answers = []
                for i in message:
                    if i == ',':
                        answers.append(float(currentvaluestr))
                        currentvaluestr = ""
                        continue
                    currentvaluestr += i
                answers.append(float(currentvaluestr))
                #print(answers)
                if not (math.isclose(answers[0], self.aggKp, abs_tol=0.00001) and math.isclose(answers[1], self.aggKi, abs_tol=0.00001) and math.isclose(answers[2], self.aggKd, abs_tol=0.00001)):
                    self.setaggPID()
            if "PID Terms: " in message:
                message = message.replace("PID Terms: ", "")
                currentvaluestr = ""
                answers = []
                for i in message:
                    if i == ',':
                        answers.append(float(currentvaluestr))
                        currentvaluestr = ""
                        continue
                    currentvaluestr += i
                answers.append(float(currentvaluestr))
                self.currentPIDTerms = answers
                #print("currentPIDTerms",currentPIDTerms)
            #self.currenterror













yogourtfermenter = YogourtFermenter()