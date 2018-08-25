#!/usr/bin/python

import threading
from time import sleep
try:
    from modules.drivers.ServoDriverController import ServoDriver
except:
    print("Could not load servo driver from turrent class")

# ============================================================================
# Turret thread that moves the servos to the correct locations corresponding
# to the coordinates of the camera.
#
# Also steps the servos in increments towards the next firing location
#  to for safety
# ============================================================================

# glocal threading event
threadexit = threading.Event()

class Controller(threading.Thread):
    def __init__(self, cfg):

        # Servo Pins
        self.triggerwait = 3
        self.servoPan = int(cfg['turret']['panchannel'])
        self.servoTilt = int(cfg['turret']['tiltchannel'])
        self.servoTrigger = int(cfg['turret']['triggerchannel'])
        self.triggerHomePos = float(cfg['turret']['triggerHomePos'])
        self.triggerFirePos = float(cfg['turret']['triggerFirePos'])
        # Driver
        self.driver = ServoDriver(cfg)
        # Behaviour variables
        # TODO add behaviour and smoothness factors
        # variables
        self.triggertimer = threading.Event()
        self.armed = False
        self.center = [0.0,0.0] #center of screen
        self.xy = self.center # current position
        self.widthPre = int(cfg['camera']['width'])
        self.heightPre = int(cfg['camera']['height'])
        self.scaledown = int(cfg['camera']['scaledown'])  # faster processing
        self.camw = int(self.widthPre / self.scaledown)
        self.camh = int(self.heightPre / self.scaledown)
        self.xMin = float(cfg['controller']['xMin'])
        self.xMax = float(cfg['controller']['xMax'])
        self.yMin = float(cfg['controller']['yMin'])
        self.yMax = float(cfg['controller']['yMax'])
        self.xRatio = (self.camw)/(self.xMax-self.xMin)
        self.yRatio = (self.camh)/(self.yMax-self.yMin)
        self.xPulse = 0.0
        self.yPulse = 0.0
        self.firing = False

        threading.Thread.__init__(self)

    def coordToPulse(self,coord):
        self.xPulse = (float(coord[0])/self.xRatio)+self.xMin
        self.yPulse = ((self.camh - float(coord[1]))/self.yRatio)+self.yMin
        return (self.xPulse,self.yPulse)

    def fire(self): # pull trigger thread
        print('skiet skiet')
        self.driver.move(self.servoTrigger,self.triggerHomePos)
        sleep(0.2)
        self.driver.move(self.servoTrigger,self.triggerFirePos)
        sleep(0.2)
        t = threading.Timer(self.triggerwait, self.triggertimer) # Timer thread that shoots for 3 seconds
        t.start()
        t.cancel() # proper termination


    def centerPosition(self):
        # TODO returns turret  to middle of screen (0,0)
        self.sendTarget()

    def sendTarget(self, newXY, curXY):
        print('Sending target')
        # TODO start stepping to new position from current pos

        if self.armed and not self.triggertimer.isSet():
            self.fire()

    def quit(self): # proper termination of thread
        global threadexit
        self.centerPosition()
        sleep(1)
        threadexit.set()

    def run(self):
        while (not threadexit.isSet()):
            # print('Turret thread running')
            # TODO Step each iteration
            sleep(0.01)
            self.xy[0] = self.yPulse
            self.xy[1] = self.xPulse
            self.driver.move(self.servoPan, self.xy[0])
            self.driver.move(self.servoTilt, self.xy[1])






