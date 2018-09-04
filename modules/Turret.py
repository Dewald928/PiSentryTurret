#!/usr/bin/python

import threading
from time import sleep
from modules.drivers.ServoDriverController import ServoDriver

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
        self.active_smoothing = True
        self.smoothing_factor = 0.7 # larger is smoother up to 1, but also slower...
        # variables
        self.triggertimer = threading.Event()
        self.armed = False
        self.firing = False
        self.center = [0.0,0.0] #center of screen
        self.xy = self.center[:] # current position
        self.oldxy = self.center[:]
        self.possiblexy = self.center[:]
        widthPre = int(cfg['camera']['width'])
        heightPre = int(cfg['camera']['height'])
        scaledown = int(cfg['camera']['scaledown'])  # faster processing
        self.camw = int(widthPre / scaledown)
        self.camh = int(heightPre / scaledown)
        self.xMin = float(cfg['controller']['xMin'])
        self.xMax = float(cfg['controller']['xMax'])
        self.yMin = float(cfg['controller']['yMin'])
        self.yMax = float(cfg['controller']['yMax'])
        self.xRatio = (self.camw)/(self.xMax-self.xMin)
        self.yRatio = (self.camh)/(self.yMax-self.yMin)
        self.xPulse = 0.0
        self.yPulse = 0.0
        self.cfg = cfg
        # test variables

        threading.Thread.__init__(self)

    def coord_to_pulse(self,coord):
        self.xPulse = (float(coord[0])/self.xRatio)+self.xMin
        self.yPulse = ((self.camh - float(coord[1]))/self.yRatio)+self.yMin
        print('Coord to pulse:', self.xPulse, self.yPulse)
        return (self.xPulse,self.yPulse)

    def fire(self): # pull trigger thread
        self.firing = True

    def reset_calibration(self):
        self.xMin = float(self.cfg['controller']['xMin'])
        self.xMax = float(self.cfg['controller']['xMax'])
        self.yMin = float(self.cfg['controller']['yMin'])
        self.yMax = float(self.cfg['controller']['yMax'])
        self.xRatio = self.camw / (self.xMax - self.xMin)
        self.yRatio = self.camh / (self.yMax - self.yMin)
        print('Calabrations are Reset')

    def flipx(self):
        oldxMin = self.xMin
        oldxMax = self.xMax
        self.xMin = oldxMax
        self.xMax = oldxMin
        self.xRatio = self.camw / (self.xMax - self.xMin)
        print('X flipped')

    def flipy(self):
        oldyMin = self.yMin
        oldyMax = self.yMax
        self.yMin = oldyMax
        self.yMax = oldyMin
        self.yRatio = (self.camh / (self.yMax - self.yMin))
        print('Y flipped')

    def center_position(self):
        # TODO returns turret  to middle of screen (0,0)
        print('Centering...')
        self.armed = False
        self.send_target(self.center, self.xy)

    def send_target(self, newXYpulse, curXYpulse):
        print('Sending target')
        # gives a new position to move towards
        self.oldxy = curXYpulse
        self.possiblexy = newXYpulse

        # fire if on target
        if self.armed and not self.triggertimer.isSet():
            self.fire()

    def quit(self): # proper termination of thread
        global threadexit
        self.center_position()
        sleep(2)
        threadexit.set()

    def run(self):
        global threadexit
        while (not threadexit.isSet()):
            # print('Turret thread running')
            # TODO Step each iteration
            # TODO anticipation
            # TODO Active smoothing
            # sleep(self.stepsleep)
            # if self.stepcounter > 0:  # stepping to target
            #     self.xy[0] += self.stepxy[0]
            #     self.driver.move(self.servoPan, self.xy[0])
            #     self.xy[1] += self.stepxy[1]
            #     self.driver.move(self.servoTilt, self.xy[1])
            #     self.stepcounter -= 1
            # else:  # set next target
            #     self.stepxy[0] = self.deltaxy[0] / self.steps
            #     self.stepxy[1] = self.deltaxy[1] / self.steps
            #     self.deltaxy = [0.0, 0.0]
            #     self.stepcounter = self.steps

            sleep(0.01)
            if self.active_smoothing:
                xdiff = self.possiblexy[0] - self.oldxy[0]
                ydiff = self.possiblexy[1] - self.oldxy[1]
                self.xy[0] = self.oldxy[0] + xdiff * (1 - self.smoothing_factor)
                self.xy[1] = self.oldxy[1] + ydiff * (1 - self.smoothing_factor)

            self.oldxy = self.xy
            self.driver.move(self.servoPan, self.xy[0])
            self.driver.move(self.servoTilt, self.xy[1])








