import time
import chime

class PIDProgram:
    def __init__(self,controller):
        self.controller = controller
        self.controller.setAllPID(12.73, .009, .1)
        minute = 60
        hour = 60 * minute
        #self.startingtemp = 80
        chime.theme('zelda')

        self.startnextsteptime = time.time()
        #self.temperatures = [68,71,78,38.5,38.5]
        self.temperatures = [82,41]
        self.temperaturetolerance = 0.3
        #self.waitsecondsaftertempreached = [0,0,5000,0,20 * hour]
        self.waitsecondsaftertempreached= [3 * hour,24*hour]
        self.effects = ["chime" for i in self.waitsecondsaftertempreached]
        self.overridepid = [False for i in self.waitsecondsaftertempreached]
        self.overridepid[0] = False
        #self.overridepid[1] = False
        #self.overridepid[3] = True

        self.currentstage = 0
        self.temperaturehasbeenreached = False
        # Pré-initialization
        self.controller.SP = self.temperatures[self.currentstage]
        self.controller.overridepid = self.overridepid[self.currentstage]
        if self.overridepid[self.currentstage]:
            self.controller.setPIDOverride()
        if not len(self.temperatures) == len(self.waitsecondsaftertempreached):
            raise Exception("self.temperatures and self.waitsecondsaftertempreached must have the same length.")

    def applyProgram(self):
        if not self.currentstage <= len(self.temperatures)-1:
            print("Program ended, Set Point is :",self.controller.SP)
            return
        # Temperature is reached
        if self.controller.currenttemp  + self.temperaturetolerance>= self.controller.SP and self.controller.currenttemp - self.temperaturetolerance <= self.controller.SP and not self.temperaturehasbeenreached:
            self.temperaturehasbeenreached = True
            self.startnextsteptime = time.time() + self.waitsecondsaftertempreached[self.currentstage]
            print("Temperature reached, next stage in : ", self.waitsecondsaftertempreached[self.currentstage],"seconds")
            if self.effects[self.currentstage] == "chime":
                chime.info()

            self.currentstage += 1

        # Temperature is reached and wait time ended
        if self.temperaturehasbeenreached and time.time() >= self.startnextsteptime and self.currentstage <= len(self.temperatures)-1:
            self.temperaturehasbeenreached = False
            self.controller.SP = self.temperatures[self.currentstage]
            self.controller.overridepid = self.overridepid[self.currentstage]
            if self.overridepid[self.currentstage]:
                self.controller.setPIDOverride()
            print("Waiting time ended, next target temperature :",self.controller.SP)
            if self.effects[self.currentstage] == "chime":
                chime.success()




