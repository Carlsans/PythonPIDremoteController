import datetime
import math
import random

import numpy as np


class OptimizePID:
    def __init__(self,controller):
        self.controller = controller
        self.SetPoint = 60
        self.controller.setSP(self.SetPoint)
        self.paramlist = ['Kp','Ki','Kd']
        self.paramtooptimize = self.paramlist[0]
        self.paramgradientstep = {'Kp':0.2,'Ki':.05,'Kd':0.1}
        self.paramstartingvalues = {'Kp': 12.73, 'Ki': .001, 'Kd': .1} # double aggKp = 3.27 * 2.56, aggKi = 0.0008* 2.56, aggKd = 646.15* 2.56;
        #self Setting new pid params for all 25.517766532192542 8.562967413771423 4.673292278251236
        #self.paramstartingvalues = {'Kp': 10.754283333333333, 'Ki': 2.5116666666666667, 'Kd': 4.005} # double aggKp = 3.27 * 2.56, aggKi = 0.0008* 2.56, aggKd = 646.15* 2.56;
        self.pauselenghtinminutes = 10

        self.startgradientstep = {'Kp':-1,'Ki':.5,'Kd':1}
        self.allparametervalues = self.paramstartingvalues
        self.totalsimulationinsecond = 60 * 40
        self.conservedsimulationinsecond = self.totalsimulationinsecond - 60 * 3
        self.maxsameparamcount = 15
        self.lastmeantempdifference = 1 # TODO
        self.directiongradient = 1
        self.currentParamValue = self.paramstartingvalues[self.paramtooptimize]
        self.lastParamValue = None
        self.sameparamoptimizationcount = 0
        self.iswaitingtoreachsetpoint = True
        self.initializeData()

    def changeParamtoOptimize(self,paramkey):
        self.paramtooptimize = paramkey
        self.lastParamValue = None
        self.currentParamValue = self.allparametervalues[paramkey]

    def __str__(self):
        returnstr = "SetPoint = " + str(self.SetPoint) +"\n Temperature list = " + str(self.TemperaturebySecList)
        returnstr += "\nSPdifferenceoverTime Mean = " + str(self.computeSPdifferenceoverTime())
        return returnstr
    def initializeData(self):

        self.TemperaturebySecList = []
        self.setPIDParams()

        #self.TemperaturebySecList = [random.randint(0,100) for i in range(self.totalsimulationinsecond -3)]
    def setSetPoint(self,sp):
        if self.SetPoint is None:
            self.SetPoint = sp
    def computeSPdifferenceoverTime(self):
        if len(self.TemperaturebySecList) == 0:
            return float('inf')
        tempdifference = [abs(self.SetPoint - i) for i in self.TemperaturebySecList]
        totaltempdifference = sum(tempdifference)
        meantempdifference = totaltempdifference / len(self.TemperaturebySecList)
        return meantempdifference
    def computeSPsquaredmeandifferenceoverTime(self):
        if len(self.TemperaturebySecList) == 0:
            return float('inf')
        tempdifference = [math.pow(abs(self.SetPoint - i),2) for i in self.TemperaturebySecList]
        totaltempdifference = sum(tempdifference)
        meantempdifference = totaltempdifference / len(self.TemperaturebySecList)
        return meantempdifference
    def optimizePID(self,temp):
        # The microcontroller must have it's temperature set before we continue.
        if not self.controller.isSPsetMCU():
            return

        if self.iswaitingtoreachsetpoint:
            if temp >= self.SetPoint:
                print("Set Point Reached at " + str(temp) + "Current SP" + str(self.SetPoint))
                self.starttime = datetime.datetime.now()
                self.iswaitingtoreachsetpoint = False
                self.controller.setSP(0)
                print("Waiting to start")
            return
        if self.controller.currentSP == 0 and self.controller.currenttemp > self.SetPoint - 5:
            print('.', end='')
            return
        # if self.starttime + datetime.timedelta(minutes=self.pauselenghtinminutes) > datetime.datetime.now():
        #     print('.', end='')
        #     #print("Waiting to start")
        #     #print(self.starttime + datetime.timedelta(minutes=self.pauselenghtinminutes), datetime.datetime.now())
        #     return
        self.controller.setSP(self.SetPoint)
        self.TemperaturebySecList.append(temp)

        if len(self.TemperaturebySecList) > self.totalsimulationinsecond:
            self.sameparamoptimizationcount += 1
            self.TemperaturebySecList = self.TemperaturebySecList[self.conservedsimulationinsecond+1:len(self.TemperaturebySecList)]
            self.computegradient()
            self.allparametervalues[self.paramtooptimize] = self.currentParamValue
            if self.sameparamoptimizationcount >= self.maxsameparamcount:
                self.sameparamoptimizationcount = 0
                if self.paramtooptimize == 'Kp':
                    self.paramtooptimize = 'Ki'
                elif self.paramtooptimize == 'Ki':
                    self.paramtooptimize = 'Kd'
                elif self.paramtooptimize == 'Kd':
                    self.paramtooptimize = 'Kp'
                print("Changing param to optimize. Now optimizing :",self.paramtooptimize)
                self.changeParamtoOptimize(self.paramtooptimize)
            self.controller.setSP(0)
            #self.starttime = datetime.datetime.now()
            self.initializeData()

    def setPIDParams(self):
        self.controller.setAllPID(self.allparametervalues['Kp'],self.allparametervalues['Ki'],self.allparametervalues['Kd'])
    def computegradient(self):
        #meantempdifference = self.computeSPdifferenceoverTime()
        meantempdifference = self.computeSPsquaredmeandifferenceoverTime()
        if self.lastParamValue is None:
            self.lastParamValue = self.currentParamValue
            self.currentParamValue += self.startgradientstep[self.paramtooptimize]
            print("First init, first K was",self.lastParamValue,"second K is",self.currentParamValue)
            print("Mean temp difference for first step",meantempdifference)
            self.lastmeantempdifference = meantempdifference
            #self.controller.setSP(0)
            return
        print("Mean temp difference is ", meantempdifference)
        slope = (meantempdifference-self.lastmeantempdifference)/(self.currentParamValue-self.lastParamValue)
        print("slope = (meantempdifference-self.lastmeantempdifference)/(self.currentParamValue-self.lastParamValue)")
        print("Slope is",slope, "(", meantempdifference,"-",self.lastmeantempdifference,")/(",self.currentParamValue,"-",self.lastParamValue,")" )
        absoluteclampedslope = np.clip(abs(slope),0.001,1)
        print("absoluteclampedslope",absoluteclampedslope)
        self.lastmeantempdifference = meantempdifference
        self.lastParamValue = self.currentParamValue
        if slope > 0:
            direction = 1
        else:
            direction = -1
        print("Direction is",direction)
        # self.currentParamValue = self.currentParamValue - absoluteclampedslope * direction * self.paramgradientstep[
        #     self.paramtooptimize]
        # print("New value after gradient descent step :", self.currentParamValue, "Step taken = ",
        #       - absoluteclampedslope * direction * self.paramgradientstep[self.paramtooptimize])

        self.currentParamValue = self.currentParamValue - self.currentParamValue * direction * self.paramgradientstep[self.paramtooptimize]
        print("New value after gradient descent step :",self.currentParamValue,"Step taken = ",- self.currentParamValue * direction * self.paramgradientstep[self.paramtooptimize])


