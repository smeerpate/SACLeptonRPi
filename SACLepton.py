#!/usr/bin/python3.7
# -*- coding: utf-8 -*-
import numpy as np
import cv2
import time
import LeptonCCI as l
import sys
from picamera.array import PiRGBArray
from picamera import PiCamera
from SACFaceFinder import FaceFinder

state = "IDLE"
faceSizeUpperLimit = 300
faceSizeLowerLimit = 200

# Initialize PiCamera
cameraWidth = 640
cameraHeight = 480
camera = PiCamera()
camera.resolution = (cameraWidth,cameraHeight)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640,480))
# allow warmup
time.sleep(0.1)

# initialize the face finder
ff = FaceFinder()

try:
	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		# get an image from the camera
		image = frame.array

		# run state machine
		if state == "IDLE":
			if ff.getTcFaceContours(image) == True:
				state = "WAIT_FOR_SIZE_OK"
			else:
				state = "IDLE"
				print("face not present")

		elif state == "WAIT_FOR_SIZE_OK":
			if ff.getTcFaceContours(image) == True:
				if ff.getTcFaceROIWidth() > faceSizeUpperLimit:
					print("Too close")
					state = "WAIT_FOR_SIZE_OK"
				elif ff.getTcFaceROIWidth() < faceSizeLowerLimit:
					print("Too far")
					state = "WAIT_FOR_SIZE_OK"
				else:
					print("Face size ok")
					state = "TEMP_OK"

			else:
				state = "IDLE"

		elif state == "TEMP_OK":
			if ff.getTcFaceContours(image) == True:
				state = "TEMP_OK"
			else:
				state = "IDLE"

		else:
			break

		# clear the stream in preparation for the next frame
		rawCapture.truncate(0)

except KeyboardInterrupt:
	print("Interuptted by user.")
