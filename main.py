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
from timeit import default_timer as timer #for checking processing speeds
from imutils.video import WebcamVideoStream
from imutils.video import FPS
import imutils

# load config
from configobj import ConfigObj  # cool read and write config file
cfg = ConfigObj(os.path.dirname(os.path.abspath(__file__)) + '/config.ini')

ix, iy = 80,60
displayframe = np.zeros((int(cfg['camera']['height']),int(cfg['camera']['width']),3), np.uint8)

# mouse click callback event
def on_click(event, cx, cy, flags, cam):
    global ix, iy, turret
    if event == cv2.EVENT_LBUTTONDOWN:
        print("Moving to pixel coordinate: " + str(cx), str(cy))
        coords = (cx,cy)
        newcoord = turret.coord_to_pulse(coords)
        currcoord = (turret.xy[0],turret.xy[1])
        turret.armed = True
        turret.send_target(newcoord,currcoord)
        # turret to position
        # fire if arrived at position
        # TODO change frame thickness for a second
    if event == cv2.EVENT_MOUSEMOVE:
        # print("Blah blah to " + str(cx), str(cy))
        ix,iy = cx,cy
        coords = (cx, cy)
        newcoord = turret.coord_to_pulse(coords)
        currcoord = (turret.xy[0], turret.xy[1])
        turret.armed = False
        turret.send_target(newcoord, currcoord)

def draw_crossair():
    global ix, iy, displayframe, cam
    cv2.circle(displayframe, (ix, iy), 5, (0, 0, 255), 1)
    cv2.line(displayframe, (0, iy), (cam.w, iy), (0, 0, 255), 1)
    cv2.line(displayframe, (ix, 0), (ix, cam.h), (0, 0, 255), 1)

class FiringThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self,name='FiringThread')
        self.daemon = True

    def run(self):
        global turret
        while True:
            if turret.firing:
                print('BANG!')
                turret.driver.move(turret.servoTrigger, turret.triggerHomePos)
                sleep(0.2) #tweak for firing speed
                turret.driver.move(turret.servoTrigger, turret.triggerFirePos)
                sleep(0.2)
                turret.firing = False
            else:
                sleep(0.01) #TODO has lag in unarmed state



# ===========================================================================================================
def main(display):
    global cam, turret, ix,iy, displayframe
    # =======================
    # ------ SETUP ----------
    # =======================
    os.system("aplay /home/pi/software/PiSentryTurret/modules/data/swvader04.wav &")

    # Create camera object
    cam = Camera.Cam(cfg) #TODO error if no camera found
    # cam.start()
    # vs = WebcamVideoStream(src=0).start()
    fading_factor = float(cfg['controller']['fading_factor'])
    min_area = int(cfg['controller']['min_area'])
    ix = int(cam.w/2)
    iy = int(cam.h/2)


    # Spawn Turret Thread (Listens and moves servos || if on target && and armed = fire)
    turret = Turret.Controller(cfg)
    fire_thread = FiringThread()
    turret.daemon = True
    turret.center_position()
    turret.start()
    fire_thread.start()

    # Spawn Controller Thread (Handles input from keyboard)
    KeyboardHandler.WaitKey().thread.start()

    #motion tracking options
    tracker = Tracker.Track(cfg, display)


    # Wait a few seconds
    print('[INFO] Starting...')
    sleep(2.5)



    #display
    if display == 1:
        cv2.namedWindow('display')
        if tracker.mode == 0:
            cv2.setMouseCallback('display', on_click, 0)


    #initialize the first and average frame
    frame = cam.read()
    frame2 = cam.read()
    avg = None
    print('[INFO] Ready!')
    # ======================================
    # ------------- LOOP -------------------
    # ======================================
    while True:
        start = timer()
        #current pulse position
        currentXY = turret.xy

        #grab current frame
        frame = cam.read()
        # frame = imutils.resize(frame, width=cam.w)
        # break if frame couldn't be captures
        if frame is None:
            print("[ERROR] First frame couldn't be captured, check webcam")
            break

        if display == 1:
            displayframe = frame


# manual mode--------------------------------------------------
        if tracker.mode == 0:
            sleep(0.01)
            draw_crossair()




# automatic mode 1 (simple background subtraction)-----------------------------------------------
        if tracker.mode == 1:
            # convert to grayscale and apply blur
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (21,21), 0) #this kernel size can be changed (5,5)

            # if the average frame is None, initialize it
            if avg is None:
                print("[INFO] Starting background frame...")
                avg = blur.copy().astype("float")
                continue

            # accumulate moving averages from the current and previous frames
            # get the difference of current and average frames
            # make all values above threshold white and others black
            cv2.accumulateWeighted(blur, avg, fading_factor)
            frame_delta = cv2.absdiff(blur, cv2.convertScaleAbs(avg))
            thresh = cv2.threshold(frame_delta, 25, 255, cv2.THRESH_BINARY)[1]

            # dilate threshold to make a blob and get the contours
            dilated = cv2.dilate(thresh, None, iterations=2)
            img, contours, _ = cv2.findContours(dilated.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # TODO test other RETR

            if len(contours) != 0:
                # get max bounding box
                rect = max(contours, key=cv2.contourArea)
                # get center of area
                M = cv2.moments(rect)
                try:
                    cx, cy = int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])
                except:
                    cx, cy = int(cam.w / 2), int(cam.h / 2)

                # print(str(cx) + " " + str(cy))
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
                #send target coordinates
                turret.send_target(turret.coord_to_pulse((cx,cy)),  currentXY)
                #TODO crossair with anticipation

# Auto mode 2 (Recurrent frames)---------------------------------------------------------

        if tracker.mode == 2:
            d = cv2.absdiff(frame, frame2)

            grey = cv2.cvtColor(d, cv2.COLOR_BGR2GRAY)

            blur = cv2.GaussianBlur(grey, (21, 21), 0)

            # if the average frame is None, initialize it
            if avg is None:
                print("[INFO] Starting background frame...")
                avg = blur.copy().astype("float")
                continue

            cv2.accumulateWeighted(blur, avg, fading_factor)
            frameDelta = cv2.absdiff(blur, cv2.convertScaleAbs(avg))

            th = cv2.threshold(frameDelta, 20, 255, cv2.THRESH_BINARY)[1]

            kernel = np.ones((3, 3), np.uint8)
            dilated = cv2.dilate(th, kernel, iterations=2)

            eroded = cv2.erode(dilated, kernel, iterations=2)

            img, contours, _ = cv2.findContours(eroded, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

            if len(contours) != 0:
                area = cv2.contourArea(max(contours, key=cv2.contourArea))
                if area > min_area:
                    rect = max(contours, key=cv2.contourArea)

                    M = cv2.moments(rect)

                    # centre of mass
                    try:
                        cx, cy = int(M['m10'] / M['m00']),int(M['m01'] / M['m00'])
                    except:
                        cx, cy = int(w / 2), int(h / 2)

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
            frame = frame2
            frame2 = cam.read()


# Auto Mode 3 (developed algorithms, slower) -------------------------------------------------------------
        if tracker.mode == 3:
            print('[WARNING]')


# display----------------------------------------------------
        if display == 1:
            cv2.imshow("display", displayframe) #creates display window
            key = cv2.waitKey(1) & 0xFF
            # transfer char from opencv window
            if key > 0:
                KeyboardHandler.keypressed.set()
                KeyboardHandler.key = chr(key)



# keyboard handler---------------------------
# ---- Calibration--------------
        if KeyboardHandler.keypressed.isSet():
            if KeyboardHandler.key == "4": # lower limit x
                turret.xMin = turret.xy[0]
                cfg['controller']['xMin'] = turret.xMin
                cfg.write()
                turret.xRatio = cam.w/(turret.xMax-turret.xMin)
                print('Minumimum x limit set to', int(((turret.xy[0]/2)*180) + 90), 'degrees', turret.xy[0])
            if KeyboardHandler.key == "6": # upper limit x
                turret.xMax = turret.xy[0]
                cfg['controller']['xMax'] = turret.xMax
                cfg.write()
                turret.xRatio = cam.w/(turret.xMax-turret.xMin)
                print('Maximum x limit set to', int(((turret.xy[0]/2)*180) + 90), 'degrees', turret.xy[0])
            if KeyboardHandler.key == "5": # lower y limit
                turret.yMin = turret.xy[1]
                cfg['controller']['yMin'] = turret.yMin
                cfg.write()
                turret.yRatio = cam.h/(turret.yMax-turret.yMin)
                print('Minumimum y limit set to', int(((turret.xy[1]/2)*180) + 90), 'degrees', turret.xy[1])
            if KeyboardHandler.key == "8": # upper x limit
                turret.yMax = turret.xy[1]
                cfg['controller']['yMax'] = turret.yMax
                cfg.write()
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
                    cv2.setMouseCallback('display', lambda *args : None)
                turret.armed = False
                tracker.mode = int(KeyboardHandler.key)
            if KeyboardHandler.key == "2": # automatic mode
                print('Automatic Mode 2 Selected')
                if display == 1:
                    cv2.setMouseCallback('display', lambda *args : None)
                turret.armed = False
                tracker.mode = int(KeyboardHandler.key)
            if KeyboardHandler.key == "3": # automatic mode
                print('[WARNING: Experimental] Automatic Mode 3 Selected')
                if display == 1:
                    cv2.setMouseCallback('display', lambda *args : None)
                turret.armed = False
                tracker.mode = int(KeyboardHandler.key)
            if KeyboardHandler.key == "f": # f arms and disarms system
                turret.armed = not turret.armed
                if turret.armed == True:
                    print('System Armed')
                else:
                    print('System Disarmed')

#  Arrow key control(manual mode) --------------------------------
            if tracker.mode == 0:
                increment = 5 # number of pixels increments
                if KeyboardHandler.key == "w":  # move up
                    iy -= increment
                    turret.send_target(turret.coord_to_pulse((ix,iy)),currentXY)
                    if iy < 0 : # out of screen limiting
                        iy = 0
                        turret.send_target(turret.coord_to_pulse((ix, iy)), currentXY)
                if KeyboardHandler.key == "s":  # move down
                    iy += increment
                    turret.send_target(turret.coord_to_pulse((ix, iy)), currentXY)
                    if iy > cam.h :
                        iy = cam.h
                        turret.send_target(turret.coord_to_pulse((ix, iy)), currentXY)
                if KeyboardHandler.key == "a":  # move left
                    ix -= increment
                    turret.send_target(turret.coord_to_pulse((ix, iy)), currentXY)
                    if ix < 0 :
                        ix = 0
                        turret.send_target(turret.coord_to_pulse((ix, iy)), currentXY)
                if KeyboardHandler.key == "d":  # move right
                    ix += increment
                    turret.send_target(turret.coord_to_pulse((ix, iy)), currentXY)
                    if ix > cam.w :
                        ix = cam.w
                        turret.send_target(turret.coord_to_pulse((ix, iy)), currentXY)
                if KeyboardHandler.key == " ":  # space for fire, because f is oopsie
                    turret.fire()

# Smoothness and Sesitivity--------------------------------------
            # TODO Smoothness calibrations

# Exit Program --------------------------------------------------
            if KeyboardHandler.key == chr(27): # quit program safely
                print("Exiting...")
                turret.quit()
                cam.stop()
                cv2.destroyAllWindows()
                break

            # reset key polling
            KeyboardHandler.WaitKey().thread.start()
            end = timer()
            # print(end-start)



    #cleanly exists program
    cv2.destroyAllWindows()
    cam.stop()
    turret.quit()


if __name__ == "__main__":
    display = 0 #default no display
    try:
        display = int(sys.argv[1])
    except:
        print('No display mode active. argv: 0 = no display, 1 = display')
    main(display)



