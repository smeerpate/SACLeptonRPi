import cv2 as cv
import random
import numpy as np
from Lepton import Lepton
from LedDriver import LedDriver
from SettingsManager import SettingsManager

class StateMachine(object):
    """description of class"""

    def __init__(self, settingsManager, ledDriver):
        self.state = None
        self.settingsManager = settingsManager
        self.ledDriver = ledDriver
        self.faceDet = cv.CascadeClassifier("/home/pi//SACLeptonRPi/haarcascade_frontalface_default.xml")
        self.lepton = Lepton()

    def run(self, image):

        print("Running state machine")
        
        settings = self.settingsManager.getSettings()
        # Steps:
        # 1) Find a face
        # 2) Check if face size is good
        # 3) If face size == good -> calibration of Lepton (FFC) + measure amb temp (currently this will be done via the lepton, later on we will use an external temp sensor)
        # 4) Get Affine coords + set ROI
        # 5) Get RIO data (can be repeated x amount of times to be sure)
        # 6) If temp < threshold -> ok
        # 7) Else -> inform the user that we are going to measure again

                        # Measure temp
        #measureTemperature(frame)
        temp = random.randint(33, 38)
        brightness = settings.brightness.value
        if temp > settings.threshold.value:
            alarmColor = settings.alarmColor
            self.ledDriver.output(alarmColor.red, alarmColor.green, alarmColor.blue, brightness)
        else:
            okColor = settings.okColor
            self.ledDriver.output(okColor.red, okColor.green, okColor.blue, brightness)

        
        global sensorWidth, sensorHeight, maxVal

        runningAvg = 0
        thSampleCount = 0
        thSampleAcc = []
        thDataValid = False
        nThSamplesToAverage = 1

        gray = cv.cvtColor(image, cv.COLOR_RGB2GRAY)
        rects = self.faceDet.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(180,180))
        faceBoxes = [(y,x+w, y+h, x) for (x,y,w,h) in rects]
        if len(faceBoxes) > 0:
            print("Detected face")
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
                cv.rectangle(image,(left,top),(right,bottom), (100,255,100), 1)

            # get thermal image from Lepton
            raw,_ = self.lepton.capture()
            # find maximum value in raw lepton data array in thRoi (face1) slice of raw data.
            thRoi = cv.boundingRect(thFace1Cnts)
            x,y,w,h = thRoi
            # x any shold not be negative. Clip the values.
            x = max(0, min(x, sensorWidth))
            y = max(0, min(y, sensorHeight))
            thRoiData = raw[y:y+h, x:x+w]
            maxVal = np.amax(thRoiData)
            print("max val: " + str(maxVal) + "in deg: " + str(float(maxVal/100.0)-273.15))
	        # get running average over N thermal samples
            #if (thSampleCount < nThSamplesToAverage):
            #    thSampleCount += 1
            #    thSampleAcc.append(maxVal)
            #else:
            #    thDataValid = True
            #    thSampleAcc.append(maxVal)
            #    runningAvg = sum(thSampleAcc)/len(thSampleAcc)
            #    print("running avg: " + str(runningAvg))
            #    maxCoord = np.where(thRoiData == maxVal)
        else:
	        # No faces found.
            runningAvg = 0
            thSampleCount = 0
            del thSampleAcc[:]
            thDataValid = False

        if thDataValid:
            # text position
            txtPosition = (500,50)

            cv.normalize(raw, raw, 0, 65535, cv.NORM_MINMAX) # extend contrast
            np.right_shift(raw, 8, raw) # fit data into 8 bits
            # draw roi if any
            if len(faceBoxes) > 0:
                x,y,w,h = thRoi
                cv.rectangle(raw, (x,y), (x+w,y+h), 255, 1)
            # make uint8 image
            thermal = np.uint8(raw)
            # convert grayscale to BGR
            thermal = cv.cvtColor(thermal, cv.COLOR_GRAY2BGR)
            color = image

        # Put data on top of the image if a face was detected.
        if len(faceBoxes) == 1 and thDataValid:
        #            measTemp = (float(maxVal/100.0)-273.15) + corrVal
            measTemp = (float(runningAvg/100.0)-273.15)
            if measTemp > feverThresh:
                cv.putText(color, "{}degC".format(measTemp), txtPosition, cv.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255,255), 2)
            else:
                cv.putText(color, "{}degC".format(measTemp), txtPosition, cv.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0,255),2)

    def reset(self):
        print("Resetting state machine")
        self.state = None


