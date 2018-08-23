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
        self.servoPan = int(cfg['turret']['panchannel'])
        self.servoTilt = int(cfg['turret']['tiltchannel'])
        self.servoTrigger = int(cfg['turret']['triggerchannel'])
        # Driver
        self.driver = ServoDriver()
        # Behaviour variables

        # variables
        self.triggertimer = threading.Event()
        self.armed = False
        self.center = [0.0,0.0] #center of screen
        self.xy = self.center # current position
        self.xMin = float(cfg['controller']['xMin'])
        self.xMax = float(cfg['controller']['xMax'])
        self.yMin = float(cfg['controller']['yMin'])
        self.yMax = float(cfg['controller']['yMax'])
        # self.xRatio =

        threading.Thread.__init__(self)

    def coordToPulse(self,coord):
        print('Hi')




