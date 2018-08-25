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
import modules.Turret as Turret
import modules.KeyboardHandler as KeyboardPoller


# load config
from configobj import ConfigObj  # cool read and write config file
cfg = ConfigObj(os.path.dirname(os.path.abspath(__file__)) + '/config.ini')


# mouse click callback event
def on_click(event, cx, cy, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print("Moving to " + str(cx), str(cy))
        coords = (cx,cy)
        newcoord = turret.coordToPulse(coords)
        currcoord = (turret.xy[0],turret.xy[1])
        turret.sendTarget(newcoord,currcoord)
        # turret to position
        # fire if arrived at position
        # TODO change frame thickness for a second





def main(display):
    global cam, turret
    # =======================
    # ------ SETUP ----------
    # =======================

    # Create camera object
    cam = Camera.Cam(cfg)
    frame1 = cam.get_frame()
    frame2 = cam.get_frame()


    # Spawn Turret Thread (Listens and moves servos || if on target && and armed = fire)
    turret = Turret.Controller(cfg)
    turret.daemon = True
    # turret.recenter()
    turret.start()
    turret.armed = True

    # TODO Spawn Controller Thread (Handles input from keyboard)
    KeyboardPoller.WaitKey().thread.start()

    # Wait a few seconds
    print('Starting up')
    i = 0
    while (i < 5):  # allow 5sec for startup
        i += 1
        sleep(0.1)



    if display == 1:
        cv2.namedWindow('display')
        cv2.setMouseCallback('display', on_click, 0)






    # ======================================
    # ------------- LOOP -------------------
    # ======================================
    while True:

        #current pulse position
        currentXY = turret.xy

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
            # draw rectangle
            x, y, w, h = cv2.boundingRect(rect)
            cv2.rectangle(frame1, (x, y), (x + w, y + h), (0, 0, 255), 1)
            cv2.putText(frame1, 'Moth Detected', (x + w + 10, y + h), 0, 0.3, (0, 0, 255))
            # TODO draw cross air
            cv2.circle(frame1, (cx, cy), 5, (0, 0, 255), 1)
            cv2.line(frame1, (0,cy), (2*w,cy), (0,0,255), 1)
            cv2.line(frame1, (cx,0), (cx,2*h), (0,0,255), 1)
            # cv2.imshow("Show", img)

            turret.sendTarget(turret.coordToPulse((cx,cy)),  currentXY)

            # TODO all of the motion things


        if display == 1:
            cv2.imshow("display", frame1)
            key = cv2.waitKey(1)
            # transfer char from opencv window
            if key > 0:
                # print(key)
                KeyboardPoller.keypressed.set()
                KeyboardPoller.key = chr(key)

        if cv2.waitKey(1) == 27:  # exit on ESC
            break

        # TODO KeyboardHandler functions
        #keyboard handler
        if KeyboardPoller.keypressed.isSet():
            if KeyboardPoller.key == "a":
                turret.xMin = turret.xy[0]
                turret.xRatio = cam.w/(turret.xMax-turret.xMin)
                print('Minumimum x limit set to', int(((turret.xy[0]/2)*180) + 90), 'degrees')
            if KeyboardPoller.key == "d":
                turret.xMax = turret.xy[0]
                turret.xRatio = cam.w/(turret.xMax-turret.xMin)
                print('Maximum x limit set to', int(((turret.xy[0]/2)*180) + 90), 'degrees')
            if KeyboardPoller.key == "s":
                turret.yMin = turret.xy[1]
                turret.yRatio = cam.h/(turret.yMax-turret.yMin)
                print('Minumimum y limit set to', int(((turret.xy[1]/2)*180) + 90), 'degrees')
            if KeyboardPoller.key == "w":
                turret.yMax = turret.xy[1]
                turret.yRatio = cam.h/(turret.yMax-turret.yMin)
                print('Maximum y limit set to', int(((turret.xy[1]/2)*180) + 90), 'degrees')
            # reset key polling
            KeyboardPoller.WaitKey().thread.start()


        frame1 = frame2
        frame2 = cam.get_frame()



    cv2.destroyAllWindows()
    cam.quit()
    turret.quit()


if __name__ == "__main__":
    display = 0 #default no display
    try:
        display = int(sys.argv[1])
    except:
        print('No display. arg 0 = no display, 1 = display (needed for calibration')
    main(display)
