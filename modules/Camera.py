#!/usr/bin/python
import cv2
import numpy as np

class Cam :
    def __init__(self, cfg):
        self.cam = cv2.VideoCapture(0)
        self.scaledown = int(cfg['camera']['scaledown']) # faster processing
        self.width = int(cfg['camera']['width'])
        self.height = int(cfg['camera']['height'])
        self.mirrored = int(cfg['camera']['mirrored'])
        self.upsidedown = int(cfg['camera']['upsidedown'])
        #sets
        self.cam.set(3, int(self.width/self.scaledown))
        self.cam.set(4, int(self.height/self.scaledown))

        self.rawframe = np.zeros((int(self.width/self.scaledown), int(self.height/self.scaledown), 3), np.uint8)

        # start thread

    def get_frame(self):
        _,self.frame = self.cam.read()
        return self.frame




    def quit(self):
        self.cam.release()