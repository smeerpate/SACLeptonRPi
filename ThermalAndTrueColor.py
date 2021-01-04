import cv2 as cv
import random
import numpy as np
import time
import sys
import LeptonCCI as l
from Lepton import Lepton
from SACOnScreenDisplay.SettingsManager import SettingsManager
from SACForeheadFinder import ForeheadFinder
from picamera.array import PiRGBArray
from picamera import PiCamera

def runFfc():
    l.RunRadFfc()

def setFluxLinearParams():
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

def getRoiFromContours(roiContours):
    # ROI Contours: LT, RT, LB, RB
    # Returns a tuple (start point, end point)
    xstart = int(roiContours[0][0])
    ystart = int(roiContours[0][1])
    xend = int(roiContours[3][0])
    yend = int(roiContours[3][1])
    #leftSide = (5, 5),(35,55)
    #rightSide = (45, 5), (75, 55)
    return (xstart, ystart), (xend, yend)

def addRectangle(image, roi, color):
    # ROI: tuple (start point, end point)
    startPoint, endPoint = roi
    cv.rectangle(image, startPoint, endPoint, color, 1)

settingsManager = SettingsManager()
roiFinder = ForeheadFinder()
settings = settingsManager.getSettings()
transformMatrix = settings.affineTransform.value
print("Affine transform:")
print(transformMatrix)
roiFinder.setTransformMatrix(transformMatrix)
lepton = Lepton()

camera = PiCamera()
camera.resolution = (640, 480)
rawCapture = PiRGBArray(camera, size=(640, 480))
camera.meter_mode = 'spot'
camera.framerate = 30
time.sleep(0.5)

for data in camera.capture_continuous(rawCapture, format="rgb", use_video_port=True):
    frame = data.array
    image = frame.copy()

    runFfc()
    #time.sleep(0.2)
    setFluxLinearParams()
    time.sleep(0.2)
    print(str(l.SetROI((37, 10, 47, 16))))
    values = l.GetROIValues()
    print('With ROI: ' + str(values[1]))

    raw,_ = lepton.capture()
    cv.normalize(raw, raw, 0, 65535, cv.NORM_MINMAX)
    np.right_shift(raw, 8, raw)
    thImage = np.uint8(raw) # 80x60
    maxTemp = np.amax(thImage)
    print('max temp = ' + str(maxTemp))
    print('max temp = ' + str((float(maxTemp/100.0)-273.15)))

    if roiFinder.getTcContours(image, settings.showFoundFace.value):
        thRoiContours = roiFinder.getThContours() # LT, RT, LB, RB
        thRoi = getRoiFromContours(thRoiContours)
        start, end = thRoi
        xTrans = 10
        thRoi = (start[0] + xTrans, start[1]), (end[0] + xTrans, end[1]) 
        addRectangle(thImage, thRoi, (255, 255, 255))
        
    x_offset=y_offset=0
    image[y_offset:y_offset+thImage.shape[0], x_offset:x_offset+thImage.shape[1]] = thImage
    imageName = "ThermalAndTrueColor"
    cv.imwrite('/home/pi/SACLeptonRPi/' + imageName +'.jpg', image)
    rawCapture.truncate(0)
    break;

rawCapture.release()