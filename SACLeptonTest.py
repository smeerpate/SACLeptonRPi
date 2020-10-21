#!/usr/bin/python3.7
# -*- coding: utf-8 -*-
import numpy as np
import cv2
import time
from Lepton import Lepton
import sys
from picamera.array import PiRGBArray
from picamera import PiCamera
import logging

# fbSize is (width,height)
def showInFrameBufferTopBottom(imageTop, imageBottom, fbSize):
    imSize = (fbSize[0], fbSize[1]/2)
    _a = np.ones((imSize[1],imSize[0]), dtype=np.uint8)*255
    _imageTop = cv2.resize(imageTop, imSize, interpolation = cv2.INTER_AREA)
    _imageBottom = cv2.resize(imageBottom, imSize, interpolation = cv2.INTER_AREA)
    # Empty framebuffer content array BGRA
    fbCont = np.zeros((screenHeight, screenWidth, 4), dtype=np.uint8)
    # Place content in fb array
    _b,_g,_r = cv2.split(_imageTop)
    _imageTop = cv2.merge((_b,_g,_r,_a))
    fbCont[0:imSize[1], 0:imSize[0]] = _imageTop
    _b,_g,_r = cv2.split(_imageBottom)
    _imageBottom = cv2.merge((_b,_g,_r,_a))
    fbCont[(fbSize[1]/2):((fbSize[1]/2)+imSize[1]), 0:imSize[0]] = _imageBottom
    # Write to frame buffer
    with open('/dev/fb0', 'rb+') as _fBuf:
        _fBuf.write(fbCont)

# Stop the cursor from blinking
#sys.stdout.write("\033[?25l")
#sys.stdout.flush()
logging.basicConfig(filename='app.log', filemode='w', format='%(name)s - %(levelname)s - %(message)s')

# Target screen is 12", 1024x768 or 768x1024 in portrait mode
screenWidth = 768
screenHeight = 1024
# Sensor size is 80x60
sensorWidth = 80
sensorHeight = 60
# Lepton offset in degC
corrVal = 0
maxVal = 0
feverThresh = 35.4

# Initialize PiCamera
cameraWidth = 640
cameraHeight = 480
camera = PiCamera()
camera.resolution = (cameraWidth,cameraHeight)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640,480))
# allow warmup
time.sleep(0.1)


################################# Initialization #########################################
# load frontal face  classifier
faceDet = cv2.CascadeClassifier("/home/pi//SACLeptonRPi/haarcascade_frontalface_default.xml")

# Initialize Lepton sensor instance.
l = Lepton()

##########################################################################################

################################ Globals #################################################
runningAvg = 0
thSampleCount = 0
thSampleAcc = []
thDataValid = False
nThSamplesToAverage = 6
##########################################################################################

############################### main loop ################################################
try:
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# get true color image from PiCamera
        image = frame.array
	# find face(s)
	gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
	rects = faceDet.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(180,180))
	faceBoxes = [(y,x+w, y+h, x) for (x,y,w,h) in rects]
	if len(faceBoxes) > 0:
	    tcFace1 = faceBoxes[0] # true color ROI
            # transform the coordinates from true color image space to thermal image space using the affine transform matrix M
            # See https://docs.opencv.org/2.4/modules/imgproc/doc/geometric_transformations.html
            M = np.array([[1.5689e-1, 8.6462e-3, -1.1660e+1],[1.0613e-4, 1.6609e-1, -1.4066e+1]])
            P_slt = np.array([[tcFace1[3]],[tcFace1[0]],[1]]) # 'slt' source left top
            P_srt = np.array([[tcFace1[1]],[tcFace1[0]],[1]]) # 'srt' source right top
            P_slb = np.array([[tcFace1[3]],[tcFace1[2]],[1]]) # 'slb' source left bottom
            P_srb = np.array([[tcFace1[1]],[tcFace1[2]],[1]]) # 'srb' source right bottom
            P_dlt = np.dot(M, P_slt)
            P_drt = np.dot(M, P_srt)
            P_dlb = np.dot(M, P_slb)
            P_drb = np.dot(M, P_srb)
            thFace1Cnts = np.array([P_dlt, P_drt, P_dlb, P_drb], dtype=np.float32)
	    for (top,right,bottom,left) in faceBoxes:
	        cv2.rectangle(image,(left,top),(right,bottom), (100,255,100), 1)

	# get thermal image from Lepton
        raw,_ = l.capture()
        # find maximum value in raw lepton data array in thRoi (face1) slice of raw data.
        if len(faceBoxes) > 0:
            thRoi = cv2.boundingRect(thFace1Cnts)
            x,y,w,h = thRoi
            # x any shold not be negative. Clip the values.
            x = max(0, min(x, sensorWidth))
            y = max(0, min(y, sensorHeight))
            thRoiData = raw[y:y+h, x:x+w]
            maxVal = np.amax(thRoiData)
	    # get running average over N thermal samples
	    if (thSampleCount < nThSamplesToAverage):
	        thSampleCount += 1
	        thSampleAcc.append(maxVal)
	    else:
	       	thDataValid = True
		thSampleAcc.append(maxVal)
	        thSampleAcc.pop(0)
	    	runningAvg = sum(thSampleAcc)/len(thSampleAcc)
            maxCoord = np.where(thRoiData == maxVal)
	else:
	    # No faces found.
	    runningAvg = 0
	    thSampleCount = 0
	    del thSampleAcc[:]
	    thDataValid = False

        # text position
        txtPosition = (500,50)

        cv2.normalize(raw, raw, 0, 65535, cv2.NORM_MINMAX) # extend contrast
        np.right_shift(raw, 8, raw) # fit data into 8 bits
        # draw roi if any
        if len(faceBoxes) > 0:
             x,y,w,h = thRoi
             cv2.rectangle(raw, (x,y), (x+w,y+h), 255, 1)
        # make uint8 image
        thermal = np.uint8(raw)
        # convert grayscale to BGR
        thermal = cv2.cvtColor(thermal, cv2.COLOR_GRAY2BGR)
        color = image

        # Put data on top of the image if a face was detected.
        if len(faceBoxes) == 1 and thDataValid:
#            measTemp = (float(maxVal/100.0)-273.15) + corrVal
	    measTemp = (float(runningAvg/100.0)-273.15)
	    if measTemp > feverThresh:
                cv2.putText(color, "{}degC".format(measTemp), txtPosition, cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255,255), 2)
            else:
                cv2.putText(color, "{}degC".format(measTemp), txtPosition, cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0,255),2)

        # show it. Prepare data for the frame buffer.
        showInFrameBufferTopBottom(color, thermal, (screenWidth,screenHeight))

        rawCapture.truncate()
        rawCapture.seek(0)
        time.sleep(1)

# stop on ctrl+C.
except KeyboardInterrupt:
	camera.close()
# There is another problem. Print out the exception.
except Exception as e:
        print(e)
	traceback.print_exc()
        camera.close()
