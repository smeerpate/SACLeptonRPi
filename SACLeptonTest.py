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
# Lepton offset in degC
corrVal = 1.45

# Initialize PiCamera
camera = PiCamera()
camera.resolution = (640,480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640,480))
# allow warmup
time.sleep(0.1)
# load frontal face  classifier
faceDet = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")

# Initialize Lepton sensor instance.
l = Lepton()

# make an alpha channel for the frame buffer image.
alpha = np.ones((screenHeight/2,screenWidth), dtype=np.uint8)*255
fbCanvas = np.zeros((screenHeight,screenWidth,4), dtype=np.uint8)

#while(1):
try:
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# get true color image from PiCamera
        image = frame.array
	# find face(s)
	gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
	rects = faceDet.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(100,100))
	boxes = [(y,x+w, y+h, x) for (x,y,w,h) in rects]
	for (top,right,bottom,left) in boxes:
	    cv2.rectangle(image,(left,top),(right,bottom), (0,255,0), 2)

	# get thermal image from Lepton
        raw,_ = l.capture()
        # find maximum value in raw lepton data array
        maxVal = np.amax(raw)
        maxCoord = np.where(raw == maxVal)
#       print(maxCoord)
#       print(maxCoord[0][0])
#       print(maxCoord[1][0])
        # text position
#        txtPosition = (maxCoord[1][0]*screenWidth/sensorWidth, maxCoord[0][0]*screenHeight/sensorHeight)
        txtPosition = (500,100)

        cv2.normalize(raw, raw, 0, 65535, cv2.NORM_MINMAX) # extend contrast
        np.right_shift(raw, 8, raw) # fit data into 8 bits
        # scale the image to full screen resolution.
        resized = cv2.resize(np.uint8(raw), (screenWidth ,screenHeight/2), interpolation = cv2.INTER_AREA)
        # convert grayscale to BGR
        thermal = cv2.cvtColor(resized, cv2.COLOR_GRAY2BGR)
        color = cv2.resize(image, (screenWidth, screenHeight/2), interpolation = cv2.INTER_AREA) 
        
        # Put interesting data on top of the image.
	
        cv2.putText(color, "{}degC".format((float(maxVal)/100.0)-273.15 + corrVal), txtPosition, cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0,255,0,255), 3)
        # show it. Prepare data for the frame buffer.
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
