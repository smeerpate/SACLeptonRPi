#!/usr/bin/python3.7
import cv2
import numpy as np
import time
from Lepton import Lepton
from picamera.array import PiRGBArray
from picamera import PiCamera

screenWidth = 768
screenHeight = 1024
aspectRatio = 4.0/3.0

camera = PiCamera()
camera.resolution = (640,480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640,480))
time.sleep(0.1)

l = Lepton()

singleOutputImageSize = (int((screenHeight/2)*aspectRatio), screenHeight/2) # (cols,rows) or (width,height) or (x,y)
alpha = np.ones((singleOutputImageSize[1],singleOutputImageSize[0]), dtype=np.uint8)*255
fbCanvas = np.zeros((screenHeight, screenWidth, 4), dtype=np.uint8)


try:
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        tcImage = frame.array
        raw,_ = l.capture()
        cv2.normalize(raw, raw, 0, 65535, cv2.NORM_MINMAX)
        np.right_shift(raw, 8, raw)
        thImage = cv2.resize(np.uint8(raw), singleOutputImageSize, interpolation = cv2.INTER_AREA)
        thImage = cv2.cvtColor(thImage, cv2.COLOR_GRAY2BGR)
        tcImage = cv2.resize(tcImage, singleOutputImageSize, interpolation = cv2.INTER_AREA)

        b,g,r = cv2.split(tcImage)
        fbImageTop = cv2.merge((b,g,r,alpha))
        fbCanvas[0:singleOutputImageSize[1], 0:singleOutputImageSize[0]] = fbImageTop

        b,g,r = cv2.split(thImage)
        fbImageBottom = cv2.merge((b,g,r,alpha))
        fbCanvas[(screenHeight/2):((screenHeight/2)+singleOutputImageSize[1]), 0:singleOutputImageSize[0]] = fbImageBottom

        with open('/dev/fb0', 'rb+') as fBuf:
            fBuf.write(fbCanvas)

        rawCapture.truncate()
        rawCapture.seek(0)

except KeyboardInterrupt:
    camera.close()
