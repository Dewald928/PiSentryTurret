#!/usr/bin/python
import cv2

class Camera :
    def __init__(self, cfg):
        self.cam = cv2.VideoCapture(0)
        self.scaledown = cfg['camera']['scaledown'] # faster processing
        self.width = cfg['camera']['width']
        self.height = cfg['camera']['height']
        self.mirrored = cfg['camera']['mirrored']
        self.upsidedown = cfg['camera']['upsidedown']
        #sets
        self.cam.set(3, int(self.width/self.scaledown))
        self.cam.set(4, int(self.height/self.scaledown))

        # start thread

    def quit(self):
        self.cam.release()