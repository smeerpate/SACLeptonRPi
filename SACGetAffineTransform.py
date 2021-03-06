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

tcCircles = []
thCircles = []

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


try:
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        tcImage = frame.array
        tcImage = cv2.cvtColor(tcImage, cv2.COLOR_BGR2GRAY)
        raw,_ = l.capture()
        cv2.normalize(raw, raw, 0, 65535, cv2.NORM_MINMAX)
        np.right_shift(raw, 8, raw)
#        thImage = cv2.resize(np.uint8(raw), singleOutputImageSize, interpolation = cv2.INTER_AREA)
#        tcImage = cv2.resize(tcImage, singleOutputImageSize, interpolation = cv2.INTER_AREA)
        thImage = np.uint8(raw) # 80x60
        tcImage = tcImage # 640x480
        # find spots in both images
        del tcCircles[:]
        del thCircles[:]
        # true color:
        thresh = cv2.threshold(tcImage, 60, 255, cv2.THRESH_BINARY_INV)[1]
        cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(cnts)==2:
            cnts = cnts[0]
        else:
            cnts = cnts[1]
        for (i, c) in enumerate(cnts):
            ((x, y), _) = cv2.minEnclosingCircle(c)
            cv2.putText(tcImage, "x:{},y:{}".format(x,y), (int(x),int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
            tcCircles.append((x,y))

        # thermal:
        thresh = cv2.threshold(thImage, 100, 255, cv2.THRESH_BINARY)[1]
        cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(cnts)==2:
            cnts = cnts[0]
        else:
            cnts = cnts[1]
        for (i, c) in enumerate(cnts):
            ((x, y), _) = cv2.minEnclosingCircle(c)
            cv2.putText(thImage, "x:{},y:{}".format(x,y), (int(x),int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
            thCircles.append((x,y))


        tcImage = cv2.cvtColor(tcImage, cv2.COLOR_GRAY2BGR)
        thImage = cv2.cvtColor(thImage, cv2.COLOR_GRAY2BGR)

        showInFrameBufferTopBottom(tcImage, thImage, (screenWidth, screenHeight))
 #       b,g,r = cv2.split(tcImage)
 #       fbImageTop = cv2.merge((b,g,r,alpha))
 #       fbCanvas[0:singleOutputImageSize[1], 0:singleOutputImageSize[0]] = fbImageTop

 #       b,g,r = cv2.split(thImage)
 #       fbImageBottom = cv2.merge((b,g,r,alpha))
 #       fbCanvas[(screenHeight/2):((screenHeight/2)+singleOutputImageSize[1]), 0:singleOutputImageSize[0]] = fbImageBottom

 #       with open('/dev/fb0', 'rb+') as fBuf:
 #           fBuf.write(fbCanvas)

        rawCapture.truncate()
        rawCapture.seek(0)
        time.sleep(3)

except KeyboardInterrupt:
    camera.close()
    print(tcCircles)
    print(thCircles)
    tri1 = np.array([np.float32(tcCircles[0]),np.float32(tcCircles[1]),np.float32(tcCircles[2])])
    tri2 = np.array([np.float32(thCircles[0]),np.float32(thCircles[1]),np.float32(thCircles[2])])
    afTrans = cv2.getAffineTransform(tri1,tri2)
    print(afTrans)
    afTrans = cv2.getAffineTransform(tri2,tri1)
    print(afTrans)
