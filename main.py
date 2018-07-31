#!/usr/bin/python3
#=====================================================
#-------------------PiSentryGun-----------------------
#---------------- By: Dewald Krynauw -----------------
#=====================================================
''' Proverbs 6:6-8
    6  Go to the ant, you sluggard!
    Consider her ways and be wise,
    7 Which, having no captain,
    Overseer or ruler,
    8 Provides her supplies in the summer,
    And gathers her food in the harvest.'''

import sys
import os
import cv2
import numpy as np
import threading
from time import sleep
import modules.Camera as Camera
import modules.Controller as Controller
import modules.Turret as Turret


# load config
from configobj import ConfigObj  # this library supports writing/saving config
cfg = ConfigObj(os.path.dirname(os.path.abspath(__file__)) + '/config.ini')


# mouse click callback event
def on_click(event, cx, cy, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print("Moving to " + str(cx), str(cy))
        # coords to position
        # turret to position
        # fire if arrived at position
    if event == cv2.EVENT_MBUTTONDOWN:
        print("FIRE!!!")







def main():
    # =======================
    # ------ SETUP ----------
    # =======================
    # camera setup
    scale = 4
    w = int(640 / scale)
    h = int(480 / scale)

    cap = cv2.VideoCapture(0)

    cap.set(3, w)
    cap.set(4, h)


    if cap.isOpened():
        ret, frame1 = cap.read()
        ret, frame2 = cap.read()
    else:
        ret = False

    display = int(cfg['camera']['display'])
    if display == 1:
        cv2.namedWindow('display')
        cv2.setMouseCallback('display', on_click, 0)




    #    print(cap.get(3))
    #    print(cap.get(4))
    import modules.drivers.ServoDriverController
    driver = modules.drivers.ServoDriverController.ServoDriver(cfg)
    driver.move(6,-0.99)



    # Spawn Camera Thread (Fetches camera frames)

    # Spawn Turret Thread (Listens and moves servos || if on target && and armed = fire)

    # Spawn Controller Thread (Handles input from keyboard)


    # ======================================
    # ------------- LOOP -------------------
    # ======================================


    while ret:

        d = cv2.absdiff(frame1, frame2)

        grey = cv2.cvtColor(d, cv2.COLOR_BGR2GRAY)

        blur = cv2.GaussianBlur(grey, (5, 5), 0)

        ret, th = cv2.threshold(blur, 20, 255, cv2.THRESH_BINARY)

        kernel = np.ones((3, 3), np.uint8)
        dilated = cv2.dilate(th, kernel, iterations=10)

        eroded = cv2.erode(dilated, kernel, iterations=10)

        img, contours, _ = cv2.findContours(eroded, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) != 0:
            rect = max(contours, key=cv2.contourArea)

            M = cv2.moments(rect)

            # centre of mass
            try:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
            except:
                cx = int(w / 2)
                cy = int(h / 2)

            print(str(cx) + " " + str(cy))
            x, y, w, h = cv2.boundingRect(rect)
            cv2.rectangle(frame1, (x, y), (x + w, y + h), (0, 0, 255), 1)       #draw rectangle
            cv2.putText(frame1, 'Moth Detected', (x + w + 10, y + h), 0, 0.3, (0, 0, 255))
            cv2.circle(frame1, (cx, cy), 5, (0, 0, 255), 1)                     #draw cross air
            # cv2.line(frame1, (0,cy), (w,cy), (0,0,255), 2)
            # cv2.line(frame1, (cx,0), (cx,h), (0,0,255), 2)
            # cv2.imshow("Show", img)


        # cv2.imshow("Original", frame2)
        if display == 1:
            cv2.imshow("display", frame1)
        if cv2.waitKey(1) == 27:  # exit on ESC
            break

        frame1 = frame2
        ret, frame2 = cap.read()

    cv2.destroyAllWindows()
    cap.release()


if __name__ == "__main__":
    main()
