#!/usr/bin/python3.7
# -*- coding: utf-8 -*-
import numpy as np
import cv2
import time
from Lepton import Lepton
import sys
from picamera.array import PiRGBArray
from picamera import PiCamera

# Stop the cursor from blinking
sys.stdout.write("\033[?25l")
sys.stdout.flush()

# Target screen is 12", 1024x768 or 768x1024 in portrait mode
screenWidth = 768
screenHeight = 1024
# Sensor size is 80x60
sensorWidth = 80
sensorHeight = 60
sensorFovH = 50 # degrees
sensorFovV = 50 # degrees
# Lepton offset in degC
corrVal = 0
maxVal = 0
feverThresh = 37.0

# Initialize PiCamera
cameraWidth = 640
cameraHeight = 480
cameraFovH = 62 # degrees
cameraFovV = 54 # degrees
camera = PiCamera()
camera.resolution = (cameraWidth,cameraHeight)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640,480))
# allow warmup
time.sleep(0.1)

# Calculate scaling factors and offsets for thermal overlay
S_x = cameraWidth / sensorWidth
S_y = cameraHeight / sensorHeight
O_u = 0
O_v = 0

################################# Initialization #########################################
# load frontal face  classifier
faceDet = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")

# Initialize Lepton sensor instance.
l = Lepton()

# make an alpha channel for the frame buffer image.
alpha = np.ones((screenHeight/2,screenWidth), dtype=np.uint8)*255
fbCanvas = np.zeros((screenHeight,screenWidth,4), dtype=np.uint8)

##########################################################################################

############################### main loop ################################################
try:
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# get true color image from PiCamera
        image = frame.array
	# find face(s)
	gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
	rects = faceDet.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(150,150))
	faceBoxes = [(y,x+w, y+h, x) for (x,y,w,h) in rects]
	if len(faceBoxes) > 0:
	    tcFace1 = rects[0] # true color ROI
            thFace1 = (int(round(rects[0][0]/S_x)+O_u), int(round(rects[0][1]/S_y)+O_v), int(round(rects[0][2]/S_x)), int(round(rects[0][3]/S_y)))
#	    print(tcFace1)
#            print(thFace1)
	    for (top,right,bottom,left) in faceBoxes:
	        cv2.rectangle(image,(left,top),(right,bottom), (100,255,100), 1)

	# get thermal image from Lepton
        raw,_ = l.capture()
        # find maximum value in raw lepton data array in thRoi (face1) slice of raw data.
        if len(faceBoxes) > 0:
            thRoi = raw[thFace1[1]:thFace1[1]+thFace1[3], thFace1[0]:thFace1[0]+thFace1[2]]
            maxVal = np.amax(thRoi)
            maxCoord = np.where(thRoi == maxVal)
#       print(maxCoord)
#       print(maxCoord[0][0])
#       print(maxCoord[1][0])
        # text position
#        txtPosition = (maxCoord[1][0]*screenWidth/sensorWidth, maxCoord[0][0]*screenHeight/sensorHeight)
        txtPosition = (500,100)

        cv2.normalize(raw, raw, 0, 65535, cv2.NORM_MINMAX) # extend contrast
        np.right_shift(raw, 8, raw) # fit data into 8 bits
        # draw roi if any
        if len(faceBoxes) > 0:
            left = thFace1[0]
            top = thFace1[1]
            right = thFace1[0]+thFace1[2]
            bottom = thFace1[1]+thFace1[3]
            cv2.rectangle(raw, (left,top), (right,bottom), 255, 1)
        # scale the image to full screen resolution.
        resized = cv2.resize(np.uint8(raw), (screenWidth ,screenHeight/2), interpolation = cv2.INTER_AREA)
        # convert grayscale to BGR
        thermal = cv2.cvtColor(resized, cv2.COLOR_GRAY2BGR)
        color = cv2.resize(image, (screenWidth, screenHeight/2), interpolation = cv2.INTER_AREA)

        # Put data on top of the image.
        measTemp = (float(maxVal/100.0)-273.15) + corrVal
	if measTemp > feverThresh:
            cv2.putText(color, "{}degC".format(measTemp), txtPosition, cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0,50,255,255), 2)
        else:
            cv2.putText(color, "{}degC".format(measTemp), txtPosition, cv2.FONT_HERSHEY_SIMPLEX, 1.2, (50,255,0,255),2)
        # show it. Prepare data for the frame buffer.
#        M =np.array([[1.306e0, -8.407e-3, -1.609e+2],[-9.533e-2, 1.287e0, -8.262e+1]])
#        color = cv2.warpAffine(color, M, (screenWidth, screenHeight/2), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT_101)
        b,g,r = cv2.split(color)
        fbImageTop = cv2.merge((b,g,r,alpha))
	fbCanvas[0:screenHeight/2, 0:screenWidth] = fbImageTop
	b,g,r = cv2.split(thermal)
	fbImageBottom = cv2.merge((b,g,r,alpha))
	fbCanvas[screenHeight/2:screenHeight,  0:screenWidth] = fbImageBottom
	# write image to the frame buffer.
        with open('/dev/fb0', 'rb+') as fBuf:
            fBuf.write(fbCanvas)

        rawCapture.truncate()
        rawCapture.seek(0)
#        time.sleep(0.1)

# stop on ctrl+C.
except KeyboardInterrupt:
	camera.close()
