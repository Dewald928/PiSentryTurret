#!/usr/bin/python
import cv2
from threading import Thread

class Cam :
    def __init__(self, cfg, name='WebcamVideoStream'):
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
        # initialize the video camera stream and read the first frame
        # from the stream
        (self.grabbed, self.frame) = self.cam.read()

        # initialize the thread name
        self.name = name

        # initialize the variable used to indicate if the thread should
        # be stopped
        self.stopped = False

    def start(self):
        # start the thread to read frames from the video stream
        t = Thread(target=self.update, name=self.name, args=())
        t.daemon = True
        t.start()
        return self

    def update(self):
        # keep looping infinitely until the thread is stopped
        while True:
            # if the thread indicator variable is set, stop the thread
            if self.stopped:
                return

            # otherwise, read the next frame from the stream
            (self.grabbed, self.frame) = self.cam.read()

    # def read(self): #enable if threading
    #     # return the frame most recently read
    #     return self.frame

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True
        self.cam.release()

    def read(self): # non threading camera read
        _, self.frame = self.cam.read()
        return self.frame

