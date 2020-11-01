#!/usr/bin/python3.7
# -*- coding: utf-8 -*-
import numpy as np
import cv2
import time
from datetime import datetime
import LeptonCCI as l
import sys
from subprocess import call
from threading import Thread
import sysv_ipc as ipc
from picamera.array import PiRGBArray
from picamera import PiCamera
from picamera.color import Color
from SACFaceFinder import FaceFinder

######### Gobals ###########
state = "IDLE"
values = (0,0,0,0)
# Settings. We will need to get these from a JSON file.
thSensorWidth = 80
thSensorHeight = 60
faceSizeUpperLimit = 280
faceSizeLowerLimit = 220
transformMatrix = np.array([[1.5689e-1, 8.6462e-3, -1.1660e+1],[1.0613e-4, 1.6609e-1, -1.4066e+1]])

# Globals for logging
currentTime = int(round(time.time()))
lastLogTime = 0
logInterval = 2 # seconds

# Globals for FFC timing
lastFFCTime = 0
needsFFC = False
maxFFCInterval = 20 # seconds.
############################


######## Functions #########
def addText(image, sMessage):
    font = cv2.FONT_HERSHEY_SIMPLEX
    org = (50, 50)
    fontScale = 1
    color = (255, 0, 0)
    thickness = 2
    image = cv2.putText(image, sMessage, org, font, fontScale, color, thickness, cv2.LINE_AA)

def checkFaceSize(image, currWidth, minWidth, maxWidth):
    if currWidth > maxWidth:
        addText(image, 'Ga wat verder staan.')
        return False
    elif currWidth < minWidth:
        addText(image, 'Ga wat dichter staan.')
        return False
    else:
        addText(image, 'Afstand OK.')
        return True

def writeLog(updateRoIValues):
    if updateRoIValues:
        line = str(int(round(time.time()))) + "," + str(l.GetAuxTemp()) + "," + str(l.GetFpaTemp()) + "," + str(l.GetROIValues()) + "," + str(l.GetROI()) + ",\n"
    else:
        line = str(int(round(time.time()))) + "," + str(l.GetAuxTemp()) + "," + str(l.GetFpaTemp()) + "," + str((0,0,0,0)) + "," + str(l.GetROI()) + ",\n"
    line = line.replace('(','')
    line =line.replace(')','')
    line = line.replace(',',';')
    line = line.replace('.',',')
    f.write(line)


# Start Displaying thread and init shared memory connection
def startDisplay():
    print("[INFO]: Starting display thread...")
    call(["./SACDisplayMixer/OGLESSimpleImageWithIPC"]) 
############################


######## main init #########
th1 = Thread(target=startDisplay)
th1.start()
time.sleep(1)

key = ipc.ftok("/home/pi/SACLeptonRPi", ord('i'))
shm = ipc.SharedMemory(key, 0, 0)
shm.attach()
print("[INFO]: Shared memory key " + str(key) + " with pointer " +  str(shm))

# Initialize PiCamera
cameraWidth = 640
cameraHeight = 480
camera = PiCamera()
camera.resolution = (cameraWidth,cameraHeight)
camera.framerate = 10

#camera.annotate_foreground = Color(y=0.4, u=-0.05, v=0.615)
#camera.annotate_text = 'Hallo!'
camera.meter_mode = 'spot'
camera.hflip = True

rawCapture = PiRGBArray(camera, size=(640,480))
# allow warmup
time.sleep(0.1)

# initialize the face finder
ff = FaceFinder()
ff.setTransformMatrix(transformMatrix)


print("[INFO]: Starting state machine...")
try:
    f = open("SACTemplog.csv", "a")
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        # get an image from the camera
        image = frame.array

        currentTime = int(round(time.time()))
        alreadyLogged = False

        # run state machine
        if state == "IDLE":
            if ff.getTcFaceContours(image) == True:
                state = "WAIT_FOR_SIZE_OK"
            else:
                state = "IDLE"
                #print("face not present")
                addText(image, 'Geen gezicht gevonden.')


        elif state == "WAIT_FOR_SIZE_OK":
            if ff.getTcFaceContours(image) == True:
                if checkFaceSize(image, ff.getTcFaceROIWidth(), faceSizeLowerLimit, faceSizeUpperLimit) == False:
                    state = "WAIT_FOR_SIZE_OK"
                else:
                    state = "RUN_FFC"
            else:
                state = "IDLE"


        elif state == "RUN_FFC":
            if currentTime > (lastFFCTime + maxFFCInterval):
                l.RunRadFfc()
                lastFFCTime = currentTime
            state = "SET_FLUX_LINEAR_PARAMS"


        elif state == "SET_FLUX_LINEAR_PARAMS":
            sensorTemp = l.GetAuxTemp()
            sceneEmissivity = 0.98
            TBkg = sensorTemp
            tauWindow = 1.0
            TWindow = sensorTemp
            tauAtm = 1.0
            TAtm = sensorTemp
            reflWindow = 0.0
            TRefl = sensorTemp
            FLParams = (sceneEmissivity,TBkg,tauWindow,TWindow,tauAtm,TAtm,reflWindow,TRefl)
            print(str(FLParams))
            l.SetFluxLinearParams(FLParams)
            state = "GET_TEMPERATURE"


        elif state == "GET_TEMPERATURE":
            thRect_x, thRect_y, thRect_w, thRect_h = cv2.boundingRect(ff.getThFaceContours())
            # x and y should not be negativeor lager then the FPA. Clip the values.
            thRect_x = max(0, min(thRect_x, thSensorWidth-2))
            thRect_y = max(0, min(thRect_y, thSensorHeight-2))
            thRect_xe = max(0, min(thRect_x + thRect_w, thSensorWidth-1))
            thRect_ye = max(0, min(thRect_y + thRect_w, thSensorHeight-1))
            thRoi = (thRect_x, thRect_y, thRect_xe, thRect_ye)
            print(str(thRoi))
            l.SetROI(thRoi)
            values = l.GetROIValues()
            print(str(values))
            writeLog(True)
            alreadyLogged = True
            state = "WAIT_FOR_NO_FACE"


        elif state == "WAIT_FOR_NO_FACE":
            if ff.getTcFaceContours(image) == True:
                state = "WAIT_FOR_NO_FACE"
                addText(image, str(values[1]) + " degC")
            else:
                state = "IDLE"



        elif state == "TEMP_OK":
            if ff.getTcFaceContours(image) == True:
                state = "TEMP_OK"
            else:
                state = "IDLE"

        else:
            break

        # display image
        try:
            im_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            im_rgb = cv2.flip(im_rgb, 0)
            shm.write(im_rgb)
        except Exception as e:
            print("[ERROR]: Can not write to shared memory: " + str(e))

        # periodically write to log
        if currentTime > lastLogTime + logInterval:
            if not alreadyLogged:
                writeLog(False)
                lastLogTime = currentTime

        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)

except KeyboardInterrupt:
    print("[INFO]: Interuptted by user.")
    f.close()
    shm.detach()
    sys.exit(-1)
    
except Exception as e:
    print("[ERROR]: Error while running state machine: " + str(e))
    f.close()
    shm.detach()
    sys.exit(-1)
