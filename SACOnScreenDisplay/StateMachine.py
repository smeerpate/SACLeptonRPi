import cv2 as cv
import random
import numpy as np
import time
import sys
import LeptonCCI as l
from Lepton import Lepton
from .LedDriver import LedDriver
from .SettingsManager import SettingsManager
from SACFaceFinder import FaceFinder
from SACForeheadFinder import ForeheadFinder

class StateMachine(object):
    """description of class"""

    def __init__(self, settingsManager, ledDriver, file):
        self.state = "IDLE"
        self.settingsManager = settingsManager
        self.ledDriver = ledDriver
        self.lepton = Lepton()
        self.logFile = file

        self.values = (0, 0, 0, 0)
        #self.roiFinder = FaceFinder()
        self.roiFinder = ForeheadFinder()
        # Settings. We will need to get these from a JSON file.
        self.thSensorWidth = 80
        self.thSensorHeight = 60
        self.faceSizeUpperLimit = 280
        self.faceSizeLowerLimit = 220
        #Affine 1
        self.transformMatrix = np.array([[1.70100612e-1, 4.91086300e-4, -2.62737066e+1],[5.51191729e-3, 1.75597084e-1, -2.09686199e+1]])
        #Affine 2
        #self.transformMatrix = np.array([[5.87940641e+0, -1.64427328e-2, 1.54129017e+2],[-1.84552050e-1, 5.69537151e+0, 1.14575214e+2]])
        self.roiFinder.setTransformMatrix(self.transformMatrix)

        # Globals for logging
        self.currentTime = int(round(time.time()))
        self.lastLogTime = 0
        self.logInterval = 2 # seconds

        # Globals for FFC timing
        self.lastFFCTime = 0
        self.needsFFC = False
        self.maxFFCInterval = 20 # seconds.

    def addText(self, image, sMessage, color):
        font = cv.FONT_HERSHEY_SIMPLEX
        org = (50, 50)
        fontScale = 1
        thickness = 2
        cv.putText(image, sMessage, org, font, fontScale, color, thickness, cv.LINE_AA)

    def addRectangle(self, image, roi, color):
        # ROI: tuple (start point, end point)
        startPoint, endPoint = roi
        cv.rectangle(image, startPoint, endPoint, color, 1)

    def writeLog(self, thRoiSet):
        line = str(int(round(time.time()))) + ";" + str(self.roiFinder.name) + ";" + str(l.GetAuxTemp()) + ";" + str(l.GetFpaTemp()) + ";" + str(l.GetROIValues()) + ";" + str(l.GetROI()) + ";" + str(thRoiSet) + ",\n"
        self.logFile.write(line)

    def checkFaceSize(self, image, currWidth, minWidth, maxWidth):
        return True
        color = (255, 0, 0)
        if currWidth > maxWidth:
            self.addText(image, 'Ga wat verder staan.', color)
            return False
        elif currWidth < minWidth:
            self.addText(image, 'Ga wat dichter staan.', color)
            return False
        else:
            self.addText(image, 'Afstand OK.', color)
            return True

    def run(self, image):

        #print("Running state machine")
        
        settings = self.settingsManager.getSettings()
        # Steps:
        # 1) Find a face
        # 2) Check if face size is good
        # 3) If face size == good -> calibration of Lepton (FFC) + measure amb temp (currently this will be done via the lepton, later on we will use an external temp sensor)
        # 4) Get Affine coords + set ROI
        # 5) Get RIO data (can be repeated x amount of times to be sure)
        # 6) If temp < threshold -> ok
        # 7) Else -> inform the user that we are going to measure again

        currentTime = int(round(time.time()))
        alreadyLogged = False
        

        if self.state == "IDLE":
            color = settings.idleColor
            self.ledDriver.output(color.red, color.green, color.blue, 100)
            if self.roiFinder.getTcContours(image, settings.showFoundFace.value) == True:
                self.state = "WAIT_FOR_SIZE_OK"
            else:
                self.state = "IDLE"
                self.addText(image, 'Geen gezicht gevonden.', (255, 0, 0))

        elif self.state == "WAIT_FOR_SIZE_OK":
            if self.roiFinder.getTcContours(image, settings.showFoundFace.value) == True:
                if self.checkFaceSize(image, self.roiFinder.getTcROIWidth(), self.faceSizeLowerLimit, self.faceSizeUpperLimit) == False:
                    self.state = "WAIT_FOR_SIZE_OK"
                else:
                    self.state = "RUN_FFC"
            else:
                self.state = "IDLE"

        elif self.state == "RUN_FFC":
            if currentTime > (self.lastFFCTime + self.maxFFCInterval):
                l.RunRadFfc()
                self.lastFFCTime = currentTime
            self.state = "SET_FLUX_LINEAR_PARAMS"

        elif self.state == "SET_FLUX_LINEAR_PARAMS":
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
            self.state = "GET_TEMPERATURE"

        elif self.state == "GET_TEMPERATURE":
            thRoiContours = self.roiFinder.getThContours() # LT, RT, LB, RB

            #thRect_x, thRect_y, thRect_w, thRect_h = cv.boundingRect(thRoi)
            # x and y should not be negativeor lager then the FPA. Clip the values.
            #thRect_x = max(0, min(thRect_x, self.thSensorWidth-2))
            #thRect_y = max(0, min(thRect_y, self.thSensorHeight-2))
            #thRect_w = max(0, min(thRect_x + thRect_w, self.thSensorWidth-1))
            #thRect_h = max(0, min(thRect_y + thRect_w, self.thSensorHeight-1))
            #thCorrected = (thRect_x, thRect_y, thRect_w, thRect_h)

            print("TH ROI Contours")
            print(str(thRoiContours))

            thRoi = self.getRoiFromContours(thRoiContours)

            # Flip thRoi vertically
            #start, end = thRoi
            #w = end[0] - start[0]
            #thRoi = (80 - start[0] - w, start[1]), (80 - start[0], end[1])

            # Translate a bit to the right
            start, end = thRoi
            xTrans = 10
            thRoi = (start[0] + xTrans, start[1]), (end[0] + xTrans, end[1])
            
            #print("Total pixels: ")
            #print(w*h)

            if settings.showFoundFace.value:
                raw,_ = self.lepton.capture()
                cv.normalize(raw, raw, 0, 65535, cv.NORM_MINMAX)
                np.right_shift(raw, 8, raw)
                thImage = np.uint8(raw) # 80x60

                self.addRectangle(thImage, thRoi, (255, 255, 255))
                #self.addRectangle(thImage, thCorrected, (255, 0, 0))
                self.roiFinder.getTcContours(image, settings.showFoundFace.value)
                x_offset=y_offset=0
                image[y_offset:y_offset+thImage.shape[0], x_offset:x_offset+thImage.shape[1]] = thImage
                imageName = "Forehead " + str(int(round(time.time())))
                cv.imwrite('/home/pi/SACLeptonRPi/' + imageName +'.jpg', image)

            self.setThRoiOnLepton(thRoi)
            
            self.values = l.GetROIValues()
            print("TH ROI from Lepton:")
            print(str(l.GetROI()))
            print(str(self.values))
            self.writeLog(thRoi)
            self.state = "WAIT_FOR_NO_FACE"

        elif self.state == "WAIT_FOR_NO_FACE":
            if self.roiFinder.getTcContours(image, settings.showFoundFace.value) == True:
                self.state = "WAIT_FOR_NO_FACE"
                temp = self.values[1]
                print("Temp: " + str(temp) + "DegC")                

                color = None

                if temp > settings.threshold.value:
                    color = settings.alarmColor                    
                else:
                    color = settings.okColor
                txt = "Temp: " + str(temp) + " " + settings.threshold.unit
                self.addText(image, txt, (color.red, color.green, color.blue))
                self.ledDriver.output(color.red, color.green, color.blue, 100)
            else:
                self.state = "IDLE"

        elif self.state == "TEMP_OK":
            if self.roiFinder.getTcContours(image, settings.showFoundFace.value) == True:
                self.state = "TEMP_OK"
            else:
                self.state = "IDLE"

       
    def setThRoiOnLepton(self, thRoi):
        # TH ROI: tuple (start point, end point)
        print("Setting ROI on Lepton:")
        print(str(thRoi))

        startPoint, endPoint = thRoi

        print(str(l.SetROI((startPoint[0], startPoint[1], endPoint[0], endPoint[1]))))

    def getRoiFromContours(self, roiContours):
        # ROI Contours: LT, RT, LB, RB
        # Returns a tuple (start point, end point)
        xstart = int(roiContours[0][0])
        ystart = int(roiContours[0][1])
        xend = int(roiContours[3][0])
        yend = int(roiContours[3][1])
        #leftSide = (5, 5),(35,55)
        #rightSide = (45, 5), (75, 55)
        return (xstart, ystart), (xend, yend)

    def reset(self):
        print("Resetting state machine")
        self.state = "IDLE"


