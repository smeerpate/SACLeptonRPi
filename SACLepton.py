#!/usr/bin/python3.7
# -*- coding: utf-8 -*-
import numpy as np
import cv2
import time
import LeptonCCI as l
import sys
from subprocess import call
from threading import Thread
import sysv_ipc as ipc
from picamera.array import PiRGBArray
from picamera import PiCamera
from picamera.color import Color
from SACFaceFinder import FaceFinder

# Gobals
state = "IDLE"
faceSizeUpperLimit = 280
faceSizeLowerLimit = 220

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


# Start Displaying thread and init shared memory connection
def startDisplay():
	print("[INFO]: Starting display thread...")
	call(["./SACDisplayMixer/OGLESSimpleImageWithIPC"])


th1 = Thread(target=startDisplay)
th1.start()
time.sleep(1)

key = ipc.ftok("/home/pi/SACLeptonRPi", ord('i'))
shm = ipc.SharedMemory(key, 0, 0)
shm.attach()
print("[INFO] Shared memory key " + str(key) + " with pointer " +  str(shm))

# Initialize PiCamera
cameraWidth = 640
cameraHeight = 480
camera = PiCamera()
camera.resolution = (cameraWidth,cameraHeight)
camera.framerate = 10

#camera.annotate_foreground = Color(y=0.4, u=-0.05, v=0.615)
#camera.annotate_text = 'Hallo!'
camera.meter_mode = 'spot'

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
			l.RunRadFfc()
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
		# clear the stream in preparation for the next frame
		rawCapture.truncate(0)

except KeyboardInterrupt:
	print("Interuptted by user.")
	shm.detach()
