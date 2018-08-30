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
import modules.KeyboardHandler as KeyboardHandler
import modules.Tracker as Tracker


# load config
from configobj import ConfigObj  # cool read and write config file
cfg = ConfigObj(os.path.dirname(os.path.abspath(__file__)) + '/config.ini')

ix, iy = 80,60
displayframe = np.zeros((int(cfg['camera']['height']),int(cfg['camera']['width']),3), np.uint8)

# mouse click callback event
def on_click(event, cx, cy, flags, cam):
    global ix, iy
    if event == cv2.EVENT_LBUTTONDOWN:
        print("Moving to pixel coordinate: " + str(cx), str(cy))
        coords = (cx,cy)
        newcoord = turret.coord_to_pulse(coords)
        currcoord = (turret.xy[0],turret.xy[1])
        turret.send_target(newcoord,currcoord)
        # turret to position
        # fire if arrived at position
        # TODO change frame thickness for a second
    if event == cv2.EVENT_MOUSEMOVE:
        # print("Blah blah to " + str(cx), str(cy))
        ix,iy = cx,cy
        coords = (cx, cy)
        # newcoord = turret.coord_to_pulse(coords)


def draw_crossair():
    global ix, iy, displayframe, cam
    cv2.circle(displayframe, (ix, iy), 5, (0, 0, 255), 1)
    cv2.line(displayframe, (0, iy), (cam.w, iy), (0, 0, 255), 1)
    cv2.line(displayframe, (ix, 0), (ix, cam.h), (0, 0, 255), 1)



def main(display):
    global cam, turret, ix,iy, displayframe
    # =======================
    # ------ SETUP ----------
    # =======================

    # Create camera object
    cam = Camera.Cam(cfg)
    frame = cam.get_frame()
    frame2 = np.zeros((int(cfg['camera']['height']),int(cfg['camera']['width']),3), np.uint8)
    ix = int(cam.w/2)
    iy = int(cam.h/2)


    # Spawn Turret Thread (Listens and moves servos || if on target && and armed = fire)
    turret = Turret.Controller(cfg)
    turret.daemon = True
    turret.center_position()
    turret.start()
    turret.armed = True

    # TODO Spawn Controller Thread (Handles input from keyboard)
    KeyboardHandler.WaitKey().thread.start()

    #motion tracking options
    tracker = Tracker.Track(cfg, display)


    # Wait a few seconds
    print('Starting up')
    i = 0
    while i < 5:  # allow 5sec for startup
        i += 1
        sleep(0.1)



    #display
    if display == 1:
        cv2.namedWindow('display')
        cv2.setMouseCallback('display', on_click, 0)


    # TODO select between manual and auto


    # ======================================
    # ------------- LOOP -------------------
    # ======================================
    while True:

        #current pulse position
        currentXY = turret.xy
        frame = cam.get_frame()
        frame2 = cam.get_frame()

        if display == 1:
            displayframe = frame


# manual mode--------------------------------------------------
        if tracker.mode == 0:
            sleep(0.05)
            draw_crossair()
            # if KeyboardHandler.keypressed.isSet():






# automatic mode 1-----------------------------------------------
        if tracker.mode == 1:

            d = cv2.absdiff(frame, frame2)

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
                overlay = displayframe.copy()
                x, y, w, h = cv2.boundingRect(rect)
                cv2.rectangle(overlay, (x, y), (x + w, y + h), (0, 0, 255), -1)
                opacity = 0.5
                cv2.addWeighted(overlay, opacity, displayframe, 1-opacity, 0 , displayframe)
                # draw cross air
                cv2.circle(displayframe, (cx, cy), 5, (0, 0, 255), 1)
                cv2.line(displayframe, (0,cy), (cam.w,cy), (0,0,255), 1)
                cv2.line(displayframe, (cx,0), (cx,cam.h), (0,0,255), 1)


                turret.send_target(turret.coord_to_pulse((cx,cy)),  currentXY)

                # TODO all of the motion things

# display----------------------------------------------------
        if display == 1:
            cv2.imshow("display", displayframe)
            key = cv2.waitKey(1)
            # transfer char from opencv window
            if key > 0:
                print(key)
                KeyboardHandler.keypressed.set()
                KeyboardHandler.key = chr(key)



# keyboard handler---------------------------
# ---- Calibration--------------
        if KeyboardHandler.keypressed.isSet():
            if KeyboardHandler.key == "4": # lower limit x
                turret.xMin = turret.xy[0]
                turret.xRatio = cam.w/(turret.xMax-turret.xMin)
                print('Minumimum x limit set to', int(((turret.xy[0]/2)*180) + 90), 'degrees', turret.xy[0])
            if KeyboardHandler.key == "6": # upper limit x
                turret.xMax = turret.xy[0]
                turret.xRatio = cam.w/(turret.xMax-turret.xMin)
                print('Maximum x limit set to', int(((turret.xy[0]/2)*180) + 90), 'degrees', turret.xy[0])
            if KeyboardHandler.key == "5": # lower y limit
                turret.yMin = turret.xy[1]
                turret.yRatio = cam.h/(turret.yMax-turret.yMin)
                print('Minumimum y limit set to', int(((turret.xy[1]/2)*180) + 90), 'degrees', turret.xy[1])
            if KeyboardHandler.key == "8": # upper x limit
                turret.yMax = turret.xy[1]
                turret.yRatio = cam.h/(turret.yMax-turret.yMin)
                print('Maximum y limit set to', int(((turret.xy[1]/2)*180) + 90), 'degrees', turret.xy[1])
            if KeyboardHandler.key == "r": # reset calibration settings
                turret.reset_calibration()
            if KeyboardHandler.key == "x": # flip x
                turret.flipx()
            if KeyboardHandler.key == "z": # flip y
                turret.flipy()
# Mode selection----------------------------------------
            if KeyboardHandler.key == "0": # manual mode
                print('Manual Mode Selected')
                if display == 1:
                    cv2.setMouseCallback('display', on_click, 0)
                tracker.mode = int(KeyboardHandler.key)
            if KeyboardHandler.key == "1": # automatic mode
                print('Automatic Mode Selected')
                if display == 1:
                    cv2.setMouseCallback('display', lambda *args : None) #TODO disable if developing
                turret.armed = False
                tracker.mode = int(KeyboardHandler.key)
            if KeyboardHandler.key == " ": # spacebar arms and disarms system
                turret.armed = not turret.armed
                # tracker.mode = 0
                if turret.armed == True:
                    print('System Armed')
                else:
                    print('System Disarmed')

#  Arrow key control(manual mode) --------------------------------
            increment = 0.1 # TODO muse coord to pulse instead, weird stuffies
            if KeyboardHandler.key == "w":  # move up
                turret.xy[1] += increment  # TODO move crossair?
                iy -= int(increment*90)
                if turret.xy[1] > 1:
                    turret.xy[1] = 0.99
            if KeyboardHandler.key == "s":  # move down
                turret.xy[1] -= increment
                iy += int(increment * 90)
                if turret.xy[1] < -1:
                    turret.xy[1] = -0.99
            if KeyboardHandler.key == "a":  # move left
                turret.xy[0] -= increment
                ix -= int(increment * 90)
                if turret.xy[0] < -1:
                    turret.xy[0] = -0.99
            if KeyboardHandler.key == "d":  # move right
                turret.xy[0] += increment
                ix += int(increment * 90)
                if turret.xy[0] > 1:
                    turret.xy[0] = 0.99
            if KeyboardHandler.key == "f":  # f for fire!, because enter isn't everything :P
                turret.fire()

# Smoothness and Sesitivity--------------------------------------
            # TODO Smoothness calibrations

# Exit Program --------------------------------------------------
            if KeyboardHandler.key == chr(27): # quit program safely
                print("Exiting...")
                turret.quit()
                cam.quit()
                cv2.destroyAllWindows()
                break

            # reset key polling
            KeyboardHandler.WaitKey().thread.start()


        frame = frame2
        frame2 = cam.get_frame()



    cv2.destroyAllWindows()
    cam.quit()
    turret.quit()


if __name__ == "__main__":
    display = 0 #default no display
    try:
        display = int(sys.argv[1])
    except:
        print('No display. argv: 0 = no display, 1 = display (needed for calibration)')
    main(display)
