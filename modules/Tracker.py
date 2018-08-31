#!/usr/bin/python

import cv2
import numpy as np
import math

# ==============================================================
# Tracking class that removes some overhead in the main file
# that will be used for motion detection.
#
# 0 - maunal mode
# 1 - automatic mode
# ==============================================================

class Track():

    def __init__(self, cfg, display):
        self.display = display
        self.mode = 0 #0-maunal, 1-auto (other modes like face detection can be added later)


