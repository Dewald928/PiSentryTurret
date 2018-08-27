#!/usr/bin/python
import cv2

class Cam :
    def __init__(self, cfg):
        self.cam = cv2.VideoCapture(0)
        self.scaledown = int(cfg['camera']['scaledown']) # faster processing
        self.widthPre = int(cfg['camera']['width'])
        self.heightPre = int(cfg['camera']['height'])
        self.mirrored = int(cfg['camera']['mirrored'])
        self.upsidedown = int(cfg['camera']['upsidedown'])
        #sets width and height of frames
        self.w = int(self.widthPre/self.scaledown)
        self.h = int(self.heightPre/self.scaledown)
        self.cam.set(3, self.w)
        self.cam.set(4, self.h)


    def get_frame(self):
        _,self.frame = self.cam.read()
        return self.frame


    def quit(self):
        self.cam.release()

