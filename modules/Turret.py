#!/usr/bin/python

import threading
from time import sleep
from modules.drivers.ServoDriverController import ServoDriver
import numpy as np

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

        # Anticipation
        self.anticipation_active = False
        self.num_pts = 10 #number of previous pulse positions
        self.ant_sens = 10 # sensitivity of anticipation
        self.pre_possible_x = [0.0] * (self.num_pts + 1)
        self.pre_possible_y = [0.0] * (self.num_pts + 1)
        self.distX = [0.0] * (self.num_pts - 1)
        self.distY = [0.0] * (self.num_pts - 1)
        self.propX = float(cfg['controller']['propX'])
        self.propY = float(cfg['controller']['propY'])
        self.antXY = [0,0] #anticipation value
        self.accX = [0.0] * (self.num_pts - 1) # acceleration between previous points
        self.accY = [0.0] * (self.num_pts - 1)

        # Behaviour variables
        self.active_smoothing = True
        self.smoothing_factor = float(cfg['turret']['smoothing_factor']) # larger is smoother up to 1, but also slower...

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

        threading.Thread.__init__(self, name='TurretThread')

    def coord_to_pulse(self,coord):
        self.xPulse = (float(coord[0])/self.xRatio)+self.xMin
        self.yPulse = ((self.camh - float(coord[1]))/self.yRatio)+self.yMin
        # print('Coord to pulse:', self.xPulse, self.yPulse)
        return (self.xPulse,self.yPulse)

    def fire(self): # pull trigger thread
        self.firing = True


    def reset_calibration(self):
        '''
        Test limits
        xmin = 33 degrees
        xmax = 138 deg
        ymin = 82deg
        ymax = 44 degs
        '''
        self.xMin = -1
        self.xMax = 1
        self.yMin = -1
        self.yMax = 1
        self.cfg['controller']['xMin'] = -1
        self.cfg['controller']['xMax'] =  1
        self.cfg['controller']['yMin'] = -1
        self.cfg['controller']['yMax'] =  1
        self.cfg.write()
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
        print('[INFO] Centering...')
        self.armed = False
        self.center[0] = (self.xMax+self.xMin)/2
        self.center[1] = (self.yMax+self.yMin)/2
        self.send_target(self.center, self.xy)
        #TODO help not really

    def anticipation(self, newXYpulse):
        #TODO check lengths

        self.pre_possible_x[0] = newXYpulse[0] #new pulse placed in 0 position
        self.pre_possible_y[0] = newXYpulse[1]

        #Acceleration between previous possible xy
        for i in range(self.num_pts - 2):
            if ((abs(self.pre_possible_x[i] - self.pre_possible_x[i+1]) < self.camw/self.ant_sens) and
                    (abs(self.pre_possible_x[i+1] - self.pre_possible_x[i+2]) < self.camw/self.ant_sens)):
                self.accX[i] = (self.pre_possible_x[i] - self.pre_possible_x[i + 1]) - (self.pre_possible_x[i + 1] - self.pre_possible_x[i + 2])

            if ((abs(self.pre_possible_y[i] - self.pre_possible_y[i + 1]) < self.camh / self.ant_sens) and
                    (abs(self.pre_possible_y[i + 1] - self.pre_possible_y[i + 2]) < self.camh / self.ant_sens)):
                self.accY[i] = (self.pre_possible_y[i] - self.pre_possible_y[i + 1]) - (self.pre_possible_y[i + 1] - self.pre_possible_y[i + 2])

        # Distance between previous possible xy/ which is also velocity?? discrete time systems are weird
        for i in range(self.num_pts -2):
            if abs(self.pre_possible_x[i] - self.pre_possible_x[i + 1]) < self.camw / self.ant_sens:
                self.distX[i] = self.pre_possible_x[i] - self.pre_possible_x[i + 1]
            else:
                self.distX[i] = 0
            if abs(self.pre_possible_y[i] - self.pre_possible_y[i + 1]) < self.camh / self.ant_sens:
                self.distY[i] = self.pre_possible_y[i] - self.pre_possible_y[i + 1]
            else:
                self.distY[i] = 0

        # Addition of speed and acceleration
        # Terms are weighed to the sensitivity

        self.antXY = [0,0]

        for i in range(self.num_pts - 2):
            self.antXY[0] = self.antXY[0] + self.distX[i] + self.accX[i]
            self.antXY[1] = self.antXY[1] + self.distY[i] + self.accY[i]

        self.antXY[0] = self.propX * self.antXY[0]
        self.antXY[1] = self.propY * self.antXY[1]

        # Move all the previous up to make space for new value at 0 position
        for i in range(self.num_pts, 0, -1):
            self.pre_possible_x[i] = self.pre_possible_x[i - 1]
            self.pre_possible_y[i] = self.pre_possible_y[i - 1]
        #TODO can't be larger than 1
        self.possiblexy[0] = np.clip(newXYpulse[0] + self.antXY[0], -1, 1)
        self.possiblexy[1] = np.clip(newXYpulse[1] + self.antXY[1], -1, 1)
        print('Anticipated pulse:', self.possiblexy)



    def send_target(self, newXYpulse, curXYpulse):
        # gives a new position to move towards
        if self.anticipation_active:
            self.anticipation(newXYpulse)
        else:
            self.possiblexy = newXYpulse

        self.oldxy = curXYpulse # TODO what impact this has on anticipation?


        # fire if on target
        if self.armed and not self.triggertimer.isSet():
            self.fire()

    def quit(self): # proper termination of thread
        global threadexit
        self.anticipation_active = False
        self.center_position()
        sleep(2)
        threadexit.set()

    def run(self):
        global threadexit
        while (not threadexit.isSet()):

            sleep(0.01)
            if self.active_smoothing:
                xdiff = self.possiblexy[0] - self.oldxy[0]
                ydiff = self.possiblexy[1] - self.oldxy[1]
                self.xy[0] = self.oldxy[0] + xdiff * (1 - self.smoothing_factor)
                self.xy[1] = self.oldxy[1] + ydiff * (1 - self.smoothing_factor)

            self.oldxy = self.xy
            self.driver.move(self.servoPan, self.xy[0])
            self.driver.move(self.servoTilt, self.xy[1])








