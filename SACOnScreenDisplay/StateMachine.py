import cv2 as cv
import random
import numpy as np
import time
import sys
import LeptonCCI as l
import paho.mqtt.client as mqtt
import struct
import random
import datetime
import pdb

from Lepton import Lepton
from .LedDriver import LedDriver
from .SettingsManager import SettingsManager
from SACFaceFinder import FaceFinder
from SACForeheadFinder import ForeheadFinder
from .DisplayMixer import DisplayMixer

def calculateTemperature(SensorSamples, FPATempSamples, SkinOffsetTemp, Coeff_A, Coeff_m):
        sensorTemp = 0
        fpaTemp = 0
        retTemp = 0
        if 1:
            print("[INFO] Calculating temperature. Sensor samples: " + str(SensorSamples) + ", FPA samples: " + str(FPATempSamples))
        if len(SensorSamples) > 0 and len(FPATempSamples) > 0:
            sensorTemp = sum(SensorSamples) / len(SensorSamples) # average SensorSamples
            fpaTemp = sum(FPATempSamples) / len(FPATempSamples) # average FPATempSamples
            retTemp = sensorTemp + (Coeff_A + fpaTemp * Coeff_m) + SkinOffsetTemp
            print("[INFO] Calculated temperature = " + str(retTemp) + " DegC.")
        else:
            print("[ERROR] Missing samples. Number of sensor samples =" + str(len(SensorSamples) + ", number of FPA samples = " + str(len(FPATempSamples)))) 
        return retTemp
        
def calculateTemperature3D(SensorSamples, FPATempSamples, SkinOffsetTemp, a, b, c):
        sensorTemp = 0
        fpaTemp = 0
        retTemp = 0
        if 1:
            print("[INFO] Calculating temperature. Sensor samples: " + str(SensorSamples) + ", FPA samples: " + str(FPATempSamples))
        if len(SensorSamples) > 0 and len(FPATempSamples) > 0:
            sensorTemp = sum(SensorSamples) / len(SensorSamples) # average SensorSamples
            fpaTemp = sum(FPATempSamples) / len(FPATempSamples) # average FPATempSamples
            retTemp = a * (fpaTemp**b) * (sensorTemp**c)
            retTemp = retTemp + SkinOffsetTemp
            print("[INFO] Calculated temperature = " + str(retTemp) + " DegC.")
        else:
            print("[ERROR] Missing samples. Number of sensor samples =" + str(len(SensorSamples) + ", number of FPA samples = " + str(len(FPATempSamples)))) 
        return retTemp
        
def calculateTemperature3DPoly(SensorSamples, FPATempSamples, SkinOffsetTemp, coeffs):
        y = 0
        x = 0
        retTemp = 0
        nCoeffs = 9
        if len(coeffs) != nCoeffs:
            print("[ERROR] Missing polynomial coefficients. Number of coefficients =" + str(len(coeffs)) + ", expected " + str(nCoeffs))
            return retTemp
        if 1:
            print("[INFO] Calculating temperature. Sensor samples: " + str(SensorSamples) + ", FPA samples: " + str(FPATempSamples))
        if len(SensorSamples) > 0 and len(FPATempSamples) > 0:
            y = sum(SensorSamples) / len(SensorSamples) # average SensorSamples
            x = sum(FPATempSamples) / len(FPATempSamples) # average FPATempSamples
            a = coeffs[0]
            b = coeffs[1]
            c = coeffs[2]
            d = coeffs[3]
            e = coeffs[4]
            f = coeffs[5]
            g = coeffs[6]
            h = coeffs[7]
            i = coeffs[8]
            retTemp = a + b*x + c*y + d*x**2 + e*x*y + f*y**2 + g*x**2*y + h*x*y**2 + i*y**3
            retTemp = retTemp + SkinOffsetTemp
            print("[INFO] Calculated temperature = " + str(retTemp) + " DegC.")
        else:
            print("[ERROR] Missing samples. Number of sensor samples =" + str(len(SensorSamples) + ", number of FPA samples = " + str(len(FPATempSamples)))) 
        return retTemp
        
def onMQTTBrokerConnect(client, userdata, flags, rc):
    if rc==0:
        client.connected_flag = True #set flag
        print("[INFO] MQTT connected OK Returned code = ",rc)
    else:
        print("[WARNING] MQTT Bad connection Returned code = ",rc)

class StateMachine(object):
    """description of class"""

    def __init__(self, settingsManager, ledDriver, displayMixer):
        self.state = "SET_INITIAL_PARAMETERS"
        self.prevState = "SET_INITIAL_PARAMETERS"
        self.settingsManager = settingsManager
        self.ledDriver = ledDriver
        self.lepton = Lepton()
        #self.logFile = file
        self.displayMixer = displayMixer
        
        ###### Settings, sort of... ######
        self.faceSizeUpperLimit = 150
        self.faceSizeLowerLimit = 70
        self.autoTrigger = False # Automatically trigger a measurement every mlAutoTriggerInterval seconds (ignoring Face ROI)
        self.autoTriggerInterval = 10 # seconds 
        self.autoTriggerAtRandomIntervals = True # Requires autoTrigger to be true. Triggers at random intervals
        self.autoTriggerRndIntervalMin = 10 # seconds
        self.autoTriggerRndIntervalMax = 30 # seconds
        self.waitAfterAutotriggerMeas = 3 # seconds, the time to stay in the WAIT FOR NO FACE state in auto trigger mode
        self.measurementInterval = 0.5 # seconds (optimal = 1)
        self.measurementIterations = 3 # times, number of measuremnts per head. (optimal = 4)
        self.maxFFCInterval = 60 # seconds. After more than maxFFCInterval, unit will do a dummy FFC
        self.maxFPATempInterval = 30 # seconds
        self.showThermalImage = True # Need to show thermal image?
        self.retriesOnResultNOK = 2 # 1 #
        self.minTempForValidMeasurement = 32 # minimum temperature for a valid measurement
        #self.alarmTempThreshold = settings.threshold.value
        self.alarmTempThreshold = 37.9
        self.settleAfterFFCInterval = 0 # seconds to settle after FFC
        self.publishMQTT = True
        self.connectionTimeoutMQTT = 5 # seconds
        self.printTemperatureOnScreen = True
        self.addThermalRoiToThermalImage = True
        self.applyTempCorrection = 3 # select correctiion method 1,2,3 or 0 for no correction
        self.SkinOffset = 2.1 #1.6 # 2.1 # 2.2 # degrees difference between timpanic temperature and forehead temp
        self.mTempCorrCoeff = 0 #0.0178 # -0.0242 # slope of the linear correction
        self.ATempCorrCoeff = 0 #1.4871 # 1.7196 # 1.725 # offset of linear correction
        # e.g. = Ref_Temp = 0.011421541195997535 * FPA_Temp^-0.08387332390433329 * Calculated_Temp^2.2833534290744972
        self.tempCorrFactor = 0.011421541195997535
        self.tempCorrFPAExp = -0.08387332390433329
        self.tempCorrSensExp = 2.2833534290744972
        self.tempCorrPolyCoeffs = (152.1478948924902, 0.9713012947694135, -15.333588606433949, 0.003675598622598923, -0.06597897304400911, 0.5198175601547528, -6.113012963569158e-05, 0.0009586603223290678, -0.005137269573036217)
        #self.tempCorrPolyCoeffs = (-2680.2643267187245, -0.14073571479380054, 213.7267184806687, 0.5524770689852628, -0.7610970264670899, -5.379862969877885, -0.01705213893779612, 0.02346792405094088, 0.041930692291342525)
        self.minMeasurementInterval = 10 # seconds. Min number of seconds between two measurements (optimaal = 20)
        self.OSDTextColor = (255,120,70)
        self.mqttBrokerAddress = "broker.hivemq.com" # "192.168.1.37" #"192.168.0.138" # "192.168.1.60"
        self.mqttBrokerPort = 1883
        self.enableTlinear = False
        ##################################
        
        self.lastAutotriggerEvent = 0
        self.lastWaitForNoFaceEntry = 0
        self.lastGetTemperatureEntry = 0
        self.measurementIterCnt = 0
        self.NOKRetryCnt = 0
        self.doSetFLParameters = False
        self.initialParametersAreSet = False
        self.lastMeasurementsArePublished = False
            
        self.temperatureSamples = []
        self.temperature = 0
        self.thSensorWidth = 80
        self.thSensorHeight = 60
        self.FPATemperatureSamples = []
        
        self.lastMeasurementTime = 0 # in ms    
        self.lastThermalFrame = []
        self.lastRawThermalFrame = []

        #self.transformMatrix = self.settingsManager.getSettings().affineTransform.value
        #self.transformMatrix = np.array([[1.63900966e-01, -2.05767606e-03, -1.99485901e+01],[1.75007260e-03, 1.66221761e-01, -1.43907617e+01]])
        #self.xTrans = 5
        with open('/home/pi/SACLeptonRPi/Calibration/AffineTransform.txt') as file:
            self.transformMatrix = [[float(digit) for digit in file.readline().split()], [float(digit) for digit in file.readline().split()]]
            self.xTrans = int(file.readline())
        print("[INFO] Loaded affine transform from settings file: " + str(self.transformMatrix))
        print("[INFO] With X Translation: " + str(self.xTrans))
        
        
        if self.autoTrigger:
            self.thRoi = (5, 5), (74, 54) # (0, 0), (79, 59) # (xstart, ystart), (xend, yend)
            self.roiFinder = None
        else:
            self.thRoi = (0, 0), (0, 0)
            self.roiFinder = ForeheadFinder()
            self.roiFinder.setTransformMatrix(self.transformMatrix)
        
        # Globals for logging
        self.currentTime = int(round(time.time()))

        # Globals for FFC timing
        self.lastFFCTime = 0 # in ms
        self.lastFPATempTime = 0
        
        # mqtt
        if self.publishMQTT:
            mqtt.Client.connected_flag = False # create flag in class
            self.mqttc = mqtt.Client()
            self.mqttc.on_connect = onMQTTBrokerConnect # bind call back function
            self.mqttc.loop_start()
            self.mqttc.connect_async(self.mqttBrokerAddress, self.mqttBrokerPort, 60)
            mqttConWaitCnt = 0
            while not self.mqttc.connected_flag: #wait in loop
                print("[INFO] Trying to connect to MQTT broker: " + self.mqttBrokerAddress)
                if mqttConWaitCnt >= self.connectionTimeoutMQTT:
                    print("[ERROR] Could not connect to MQTT broker: " + self.mqttBrokerAddress)
                    break
                mqttConWaitCnt = mqttConWaitCnt + 1
                time.sleep(1)
            #self.mqttc.loop_stop() # no need for continuously listening


    def addText(self, image, sMessage, color):
        font = cv.FONT_HERSHEY_SIMPLEX
        org = (50, 50)
        fontScale = 1
        thickness = 2
        cv.putText(image, sMessage, org, font, fontScale, color, thickness, cv.LINE_AA)
    
    
    ###########################################
    # Adds a capture from the lepton to the
    # image. The thermal image is normalized.
    # 
    ###########################################
    def addThermalImage(self, image, roi, addRoi):
        if len(self.lastThermalFrame) == self.thSensorHeight * self.thSensorWidth:
            thRaw = np.reshape(self.lastThermalFrame, (self.thSensorHeight, self.thSensorWidth))
            cv.normalize(thRaw, thRaw, 0, 65535, cv.NORM_MINMAX) # extend contrast
            np.right_shift(thRaw, 8, thRaw) # fit raw data into 8 bit
            # make uint8 image
            thermal = np.uint8(thRaw)
        else:
            print("[ERROR] addThermalImage: wrong number of pixels. Got " + str(len(self.lastThermalFrame)) + " expected " + str(self.thSensorHeight * self.thSensorWidth))
            thermal = np.full((self.thSensorHeight, self.thSensorWidth), 100, dtype=np.uint8)
        # convert grayscale to pseudo color BGR
        if 1:
            thermal = cv.applyColorMap(thermal, cv.COLORMAP_HOT)
        else:
            thermal = cv.applyColorMap(thermal, cv.COLORMAP_JET)
        if addRoi:
            self.addRectangle(thermal, roi, (255,0,0))
        xOffset = yOffset = 10
        scale = 0.3
        backgnd = image.copy()
        thermal = cv.resize(thermal, (int(scale*640)-xOffset, int(scale*480)-yOffset))
        backgnd[yOffset:yOffset+thermal.shape[0], xOffset:xOffset+thermal.shape[1]] = thermal
        #return thermal
        return backgnd

    def addRectangle(self, image, roi, color):
        # ROI: tuple (start point, end point)
        startPoint, endPoint = roi
        cv.rectangle(image, startPoint, endPoint, color, 1)

    def writeLog(self):
        line = str(int(round(time.time()))) + ";" + str(self.roiFinder.name) + ";" + str(l.GetAuxTemp()) + ";" + str(l.GetFpaTemp()) + ";" + str(l.GetROIValues()) + ";" + str(l.GetROI()) + ";" + str(self.thRoi) + ";" + str(int(round(self.lastFFCTime))) + "\n"
        print("SAC_TEMPLOG: " + line)
        #self.logFile.write(line)
        

    def run(self, image):
        # save last state
        self.prevState = self.state
        # Get current time
        smEntryTimeMs = int(round(time.time() * 1000))
        try:
            # Update settings
            settings = self.settingsManager.getSettings()

            # Steps:
            # 1) Find a face
            # 2) Check if face size is good
            # 3) If face size == good -> calibration of Lepton (FFC) + measure amb temp (currently this will be done via the lepton, later on we will use an external temp sensor)
            # 4) Get Affine coords + set ROI
            # 5) Get RIO data (can be repeated x amount of times to be sure)
            # 6) If temp < threshold -> ok
            # 7) Else -> inform the user that we are going to measure again

            if self.state == "IDLE":
                #pdb.set_trace()
                #color = settings.idleColor
                self.ledDriver.output(0, 0, 0, 100)
                self.NOKRetryCnt = 0 # reset the measurement retry counter
                self.measurementIterCnt = 0 # reset iteration counter
                if self.autoTrigger:
                    if smEntryTimeMs > (self.lastAutotriggerEvent + (self.autoTriggerInterval * 1000)):
                        # Do Auto trigger trigger
                        self.state = "RUN_FFC"
                        self.lastAutotriggerEvent = smEntryTimeMs
                        dateValue = datetime.datetime.fromtimestamp(time.time())
                        print("[INFO] Autotriggering measurement. (Timestamp = " + dateValue.strftime('%Y-%m-%d %H:%M:%S') + ")")
                    else:
                        # did we already set the initial parameters?
                        if self.initialParametersAreSet:
                            self.state = "IDLE"
                        else:
                            self.state = "SET_INITIAL_PARAMETERS"
                        
                        self.currentTime = int(round(time.time() * 1000))
                        if self.currentTime > (self.lastFPATempTime + (self.maxFPATempInterval * 1000)):
                            self.getFpaTemp()
                            self.lastFPATempTime = self.currentTime
                            self.publishFPATemp()
                else:
                    if self.roiFinder.getTcContours(image, settings.showFoundFace.value):
                        self.state = "SETTLE_AFTER_MEASURING"
                        self.displayMixer.showDontMove(image)
                        dateValue = datetime.datetime.fromtimestamp(time.time())
                        print("[INFO] Found a face. (Timestamp = " + dateValue.strftime('%Y-%m-%d %H:%M:%S') + ")")
                    else:
                        # No face found.
                        self.displayMixer.hide()
                        
                        # did we already set the initial parameters?
                        if self.initialParametersAreSet:
                            self.state = "IDLE"
                        else:
                            self.state = "SET_INITIAL_PARAMETERS"
                        if smEntryTimeMs > (self.lastFFCTime + (self.maxFFCInterval * 1000)):
                            # it's been too long a time since we've done an FFC
                            self.state = "DO_DUMMY_FFC"
                            
                        self.currentTime = int(round(time.time() * 1000))
                        if self.currentTime > (self.lastFPATempTime + (self.maxFPATempInterval * 1000)):
                            self.getFpaTemp()
                            self.lastFPATempTime = self.currentTime
                            self.publishFPATemp()
                    #print("Idle took: " + str(int(round(time.time() * 1000)) - smEntryTimeMs) + "ms")

     
            elif self.state == "SET_INITIAL_PARAMETERS":
                self.setFluxLinearParams()
                if self.enableTlinear:
                    self.setRadTLinearEnableState(1)
                else:
                    self.setRadTLinearEnableState(0)
                self.setFfcShutterModeAuto(0)
                self.initialParametersAreSet = True
                print("[INFO] Initial parametars are set.")
                self.state = "IDLE"
                
                
            elif self.state == "SETTLE_AFTER_MEASURING":
                if smEntryTimeMs > (self.lastFFCTime + (self.minMeasurementInterval * 1000)):
                #if smEntryTimeMs > (self.lastGetTemperatureEntry + (self.minMeasurementInterval * 1000)): 
                    self.state = "WAIT_FOR_SIZE_OK"                        
                else:
                    if self.roiFinder.getTcContours(image, settings.showFoundFace.value):
                        self.state = "SETTLE_AFTER_MEASURING"
                        self.displayMixer.showSettling(image)
                    else:
                        self.state = "IDLE"
                        self.displayMixer.hide()
                
            elif self.state == "DO_DUMMY_FFC":
                print("[INFO] Doing dummy FFC ...")
                self.runFfc()
                self.state = "IDLE"
                
                
            elif self.state == "WAIT_FOR_SIZE_OK":
                if self.roiFinder.getTcContours(image, settings.showFoundFace.value):
                    #if self.checkFaceSize(image, self.roiFinder.getTcROIWidth(), self.faceSizeLowerLimit, self.faceSizeUpperLimit) == -1:
                    if self.roiFinder.getTcROIWidth() < self.faceSizeLowerLimit:
                        # too far
                        self.state = "WAIT_FOR_SIZE_OK"
                        self.displayMixer.showStepCloser(image)
                    #elif self.checkFaceSize(image, self.roiFinder.getTcROIWidth(), self.faceSizeLowerLimit, self.faceSizeUpperLimit) == 1:
                    elif self.roiFinder.getTcROIWidth() > self.faceSizeUpperLimit:
                        # too close
                        self.state = "WAIT_FOR_SIZE_OK"
                        self.displayMixer.showStepBack(image)
                    else:
                        # size is OK
                        self.currentTime = int(round(time.time()))
                        self.state = "RUN_FFC"
                        dateValue = datetime.datetime.fromtimestamp(time.time())
                        print("[INFO] Starting measurement. (Timestamp = " + dateValue.strftime('%Y-%m-%d %H:%M:%S') + ")")
                        self.displayMixer.showDontMove(image)
                else:
                    print("[WARNING] Face dissapeared while waiting for size OK.")
                    self.resetStateMachine()
                #print("Wait for size took: " + str(int(round(time.time() * 1000)) - smEntryTimeMs) + "ms")


            elif self.state == "RUN_FFC":
                #start = int(round(time.time() * 1000))
                if self.autoTrigger:
                    self.displayMixer.showDontMove(image)
                    self.runFfc()   
                    self.state = "SET_FLUX_LINEAR_PARAMS" 
                else:    
                    if self.roiFinder.getTcContours(image, settings.showFoundFace.value):
                        self.displayMixer.showDontMove(image)
                        self.runFfc()    
                        # maybe add a delay here 
                        self.state = "SET_FLUX_LINEAR_PARAMS"                
                    else:
                        self.state = "IDLE"
                        self.temperatureSamples = []
                        self.FPATemperatureSamples = []
                        self.displayMixer.hide()
                print("[INFO] Run FFC took: " + str(int(round(time.time() * 1000)) - smEntryTimeMs) + "ms")
                print("[INFO] Settling " + str(self.settleAfterFFCInterval) + " seconds before measuring.")


            elif self.state == "SET_FLUX_LINEAR_PARAMS":   
                #start = int(round(time.time() * 1000))
                if self.autoTrigger:
                    self.displayMixer.showMeasuring(image)
                    if self.doSetFLParameters:
                        self.setFluxLinearParams()
                    if self.settleAfterFFCInterval != 0:
                        self.state = "SETTLTE_AFTER_FFC"
                    else:
                        self.state = "GET_TEMPERATURE"
                else:
                    if self.roiFinder.getTcContours(image, settings.showFoundFace.value) or self.autoTrigger:
                        self.displayMixer.showMeasuring(image)
                        if self.doSetFLParameters:
                            self.setFluxLinearParams()
                        if self.settleAfterFFCInterval != 0:
                            self.state = "SETTLTE_AFTER_FFC"
                        else:
                            self.state = "GET_TEMPERATURE"               
                    else:
                        self.state = "IDLE"
                        self.temperatureSamples = []
                        self.FPATemperatureSamples = []
                        self.displayMixer.hide()
                #print("Set flux linear params took: " + str(int(round(time.time() * 1000)) - smEntryTimeMs) + "ms")
                #self.NOKRetryCnt = 0 # reset the measurement retry counter
                #self.measurementIterCnt = 0 # reset iteration counter
                self.doSetFLParameters = False # only set Flux liear parameters once
                
            
            elif self.state == "SETTLTE_AFTER_FFC":
                if smEntryTimeMs > (self.lastFFCTime + (self.settleAfterFFCInterval * 1000)):
                    self.state = "GET_TEMPERATURE"
                else:
                    self.state = "SETTLTE_AFTER_FFC"


            elif self.state == "GET_TEMPERATURE":
                if self.autoTrigger:
                    self.temperatureSamples.append(self.measureTemp(self.thRoi))
                    self.FPATemperatureSamples.append(self.getFpaTemp())
                    # register time
                    self.lastGetTemperatureEntry = smEntryTimeMs
                    # increment iteration counter
                    self.measurementIterCnt = self.measurementIterCnt + 1
                    # did we have all iterations?
                    if self.measurementIterCnt > self.measurementIterations:
                        self.state = "EVALUATE_RESULT"
                    else:
                        self.state = "WAIT_TO_TAKE_NEXT_SAMPLE"
                        print("[INFO] Waiting to get next sample ... Current sample was:  " + str(self.measurementIterCnt))
                else:
                # todo implement retries
                    if self.roiFinder.getTcContours(image, settings.showFoundFace.value):  
                        thRoiContours = self.roiFinder.getThContours() # LT, RT, LB, RB

                        #thRect_x, thRect_y, thRect_w, thRect_h = cv.boundingRect(thRoi)
                        # x and y should not be negativeor lager then the FPA. Clip the values.
                        #thRect_x = max(0, min(thRect_x, self.thSensorWidth-2))
                        #thRect_y = max(0, min(thRect_y, self.thSensorHeight-2))
                        #thRect_w = max(0, min(thRect_x + thRect_w, self.thSensorWidth-1))
                        #thRect_h = max(0, min(thRect_y + thRect_w, self.thSensorHeight-1))
                        #thCorrected = (thRect_x, thRect_y, thRect_w, thRect_h)

                        #print("TH ROI Contours")
                        #print(str(thRoiContours))
                        
                        # update the thermal ROI
                        self.thRoi = self.getRoiFromContours(thRoiContours)

                        # Flip thRoi vertically
                        #start, end = thRoi
                        #w = end[0] - start[0]
                        #thRoi = (80 - start[0] - w, start[1]), (80 - start[0], end[1])

                        # Translate a bit to the right
                        start, end = self.thRoi
                        self.thRoi = (start[0] + self.xTrans, start[1]), (end[0] + self.xTrans, end[1])

                        #self.setThRoiOnLepton(self.thRoi)
                        #self.writeLog()
                        #values = l.GetROIValues()
                        #self.temperatureSamples.append(values[1])
                        if self.showThermalImage:
                            self.temperatureSamples.append(self.measureTemp(self.thRoi))
                        self.FPATemperatureSamples.append(self.getFpaTemp())
                        # register time
                        self.lastGetTemperatureEntry = smEntryTimeMs
                        # increment iteration counter
                        self.measurementIterCnt = self.measurementIterCnt + 1
                        # did we have all iterations?
                        if self.measurementIterCnt > self.measurementIterations:
                            self.state = "EVALUATE_RESULT"
                        else:
                            self.state = "WAIT_TO_TAKE_NEXT_SAMPLE"
                            print("[INFO] Waiting to get next sample ... Current sample was:  " + str(self.measurementIterCnt))
                    else:
                        print("[WARNING] Face dissapeared while measuring.")
                        self.resetStateMachine()
                        #self.state = "IDLE"
                        #self.temperatureSamples = []
                        #self.FPATemperatureSamples = []
                        #self.displayMixer.hide()
                #print("Get temp took: " + str(int(round(time.time() * 1000)) - startTime) + "ms")
                
                
                    
            
            
            elif self.state == "WAIT_TO_TAKE_NEXT_SAMPLE":
                if smEntryTimeMs > (self.lastGetTemperatureEntry + (self.measurementInterval * 1000)):
                    self.state = "GET_TEMPERATURE"
                else:
                    self.state = "WAIT_TO_TAKE_NEXT_SAMPLE"

                
            elif self.state == "EVALUATE_RESULT":
                if self.applyTempCorrection == 1:
                    # using correction method #1
                    self.temperature = calculateTemperature(self.temperatureSamples, self.FPATemperatureSamples, self.SkinOffset, self.ATempCorrCoeff, self.mTempCorrCoeff)
                elif self.applyTempCorrection == 2:
                    # using correction method #2
                    self.temperature = calculateTemperature3D(self.temperatureSamples, self.FPATemperatureSamples, self.SkinOffset, self.tempCorrFactor, self.tempCorrFPAExp, self.tempCorrSensExp)
                elif self.applyTempCorrection == 3:
                    # using correction method #3
                    self.temperature = calculateTemperature3DPoly(self.temperatureSamples, self.FPATemperatureSamples, self.SkinOffset, self.tempCorrPolyCoeffs)
                else:
                    # No correction
                    self.temperature = calculateTemperature(self.temperatureSamples, self.FPATemperatureSamples, 0, 0, 0)
                    
                self.lastMeasurementsArePublished = False
                if self.showThermalImage:
                    if 1:
                        print("[INFO] Showing thermal image...")
                    image = self.addThermalImage(image, self.thRoi, self.addThermalRoiToThermalImage)
                if self.printTemperatureOnScreen:
                    if 1:
                            print("[INFO] Printing text to screen...")
                    self.addText(image, "{:.1f}".format(self.temperature + 1), self.OSDTextColor)
                if self.temperature > self.alarmTempThreshold:
                    if self.NOKRetryCnt < self.retriesOnResultNOK:
                        # OK, bad measurement: retry measuring
                        print("[INFO] Temperature was over threshold (" + str(self.temperature) + "DegC), retrying.")
                        self.measurementIterCnt = 0 # reset iteration counter
                        self.temperatureSamples = []
                        self.FPATemperatureSamples = []
                        #self.state = "GET_TEMPERATURE"
                        self.state = "RUN_FFC"
                        self.NOKRetryCnt = self.NOKRetryCnt + 1
                    else:
                        if 1:
                            print("[WARNING] Temperature was over threshold after " + str(self.NOKRetryCnt) + " retries.")
                        color = settings.alarmColor
                        self.ledDriver.output(color.red, color.green, color.blue, 100)
                        self.displayMixer.showTemperatureNok(image)
                        self.state = "WAIT_FOR_NO_FACE"                  
                elif self.temperature < self.minTempForValidMeasurement:
                    if self.NOKRetryCnt < self.retriesOnResultNOK:
                        # OK, bad measurement: retry measuring
                        print("[INFO] Measured temperature (" + str(self.temperature) + "DegC) was not valid, retrying.")
                        self.measurementIterCnt = 0 # reset iteration counter
                        self.temperatureSamples = []
                        self.FPATemperatureSamples = []
                        #self.state = "GET_TEMPERATURE"
                        self.state = "RUN_FFC"
                        self.NOKRetryCnt = self.NOKRetryCnt + 1
                    else:
                        if 1:
                            print("[WARNING] Temperature was below valid value after " + str(self.NOKRetryCnt) + " retries.")
                        color = settings.alarmColor
                        self.displayMixer.showTemperatureNok(image)
                        self.state = "WAIT_FOR_NO_FACE"
                else:
                    if 1:
                        print("[INFO] Temperature is within bounds.")
                    color = settings.okColor
                    self.ledDriver.output(color.red, color.green, color.blue, 100)
                    self.displayMixer.showTemperatureOk(image)
                    self.state = "WAIT_FOR_NO_FACE"
                
                
            elif self.state == "WAIT_FOR_NO_FACE":
                if not self.lastMeasurementsArePublished:
                    self.publishMeasurements()
                    self.lastMeasurementsArePublished = True
                    
                if self.autoTrigger:
                    time.sleep(self.waitAfterAutotriggerMeas)
                    self.resetStateMachine()       
                    if self.autoTriggerAtRandomIntervals:
                        self.autoTriggerInterval = random.randint(self.autoTriggerRndIntervalMin, self.autoTriggerRndIntervalMax)
                        print("[INFO] Random intervals set, next measurement in " + str(self.autoTriggerInterval) + " seconds.")
                else:
                    self.roiFinder.getTcContours(image, False) # only looks for the head now
                    if self.roiFinder.faceFound:   
                        if self.showThermalImage:
                            thImage = self.addThermalImage(image, self.thRoi, self.addThermalRoiToThermalImage)
                            if self.printTemperatureOnScreen:
                                if 1:
                                        print("[INFO] Printing text to screen...")
                                print("Measured " + str(self.temperature))
                                tempToShow = self.temperature + 1
                                print("Showing " + str(tempToShow))
                                self.addText(thImage, "{:.1f}".format(tempToShow), self.OSDTextColor)
                            if self.temperature > self.alarmTempThreshold:
                                self.displayMixer.showTemperatureNok(thImage)
                            else:
                                self.displayMixer.showTemperatureOk(thImage)
                        else:
                            if self.printTemperatureOnScreen:
                                if 1:
                                        print("[INFO] Printing text to screen...")
                                self.addText(image, "{:.1f}".format(self.temperature + 1), self.OSDTextColor)
                            if self.temperature > self.alarmTempThreshold:
                                self.displayMixer.showTemperatureNok(image)
                            else:
                                self.displayMixer.showTemperatureOk(image)
                    else:
                        self.resetStateMachine()

        except Exception as e:
            print("[ERROR] Main state machine error. State = " + str(self.state) + ". -> " + str(e))
            self.resetStateMachine()
        

    def measureTemp(self, roi):
        print("[INFO] ROI on thermal image is " + str(roi) + ". Reading framebuffer...")
        thFrame = l.GetFrameBuffer("/dev/spidev0.0")
        self.lastRawThermalFrame = thFrame
        if type(thFrame) is str:
            # GetFrameBuffer returns a string on error
            print("[ERROR] " + thFrame)
            return 0
        else:
            if len(thFrame) == self.thSensorHeight * self.thSensorWidth:
                self.lastThermalFrame = np.array(thFrame)
                thFrame = np.reshape(thFrame, (self.thSensorHeight, self.thSensorWidth))
                self.lastMeasurementTime = (int(round(time.time() * 1000)))
                # roi is ((x_start, y_start), (x_end, y_end))
                x_start = int(max(0, min(roi[0][0], self.thSensorWidth-1)))
                x_end = int(max(0, min(roi[1][0], self.thSensorWidth-1)))
                y_start = int(max(0, min(roi[0][1], self.thSensorHeight-1)))
                y_end = int(max(0, min(roi[1][1], self.thSensorHeight-1)))
                thFrameRoi = thFrame[y_start:y_end, x_start:x_end]
                if self.enableTlinear:
                    tRet = (np.max(thFrameRoi)/100) - 273.15
                else:
                    tRet = np.max(thFrameRoi)/100
                if 1:
                    print("[INFO] Maximum temperature in ROI (x: " + str(x_start) + ", " + str(x_end) + ", y: " + str(y_start) + ", " + str(y_end) + ") on thermal image is " + str(tRet) + "°C")
                return tRet
            else:
                print("[ERROR] Not enough pixels from thermal imaging sensor. Got" + len(self.thFrame))
                return 0;
        
    def publishMeasurements(self):
        if self.publishMQTT:
            # Get FPA and AUX temp an put it in a csv line like this:
            # "Current time;AUX Temp;FPA Temp;Last FFC time;"
            sMQTTMessage = str(int(round(time.time() * 1000))) + ';' + str(l.GetAuxTemp()) + ';' + str(l.GetFpaTemp()) + ';' + str(self.lastFFCTime) + ';' + str(self.temperature) + ";"
            sMQTTMessage = sMQTTMessage.replace('.',',')
            self.mqttc.publish("SAC_V5_LeberKaese0001/TempCx100", sMQTTMessage)
            #thFrame = l.GetFrameBuffer("/dev/spidev0.0")
            thFrame = self.lastRawThermalFrame
            if type(thFrame) is str:
                # GetFrameBuffer returns a string on error
                print("[ERROR] thFrame is of unexpected type 'string'. " + thFrame)
            else:
                if 0:
                    print("[INFO] Framebuffer length is: " + str(len(thFrame)))
                # publish the frame
                self.mqttc.publish("SAC_V5_LeberKaese0001/FrameBuffer", struct.pack('%sH' %len(thFrame), *thFrame))
        else:
            pass
            
    def publishFPATemp(self):
        if self.publishMQTT:
            # Get FPA and AUX temp an put it in a csv line like this:
            # "Current time;AUX Temp;FPA Temp;Last FFC time;"
            sMQTTMessage = str(l.GetFpaTemp())
            sMQTTMessage = sMQTTMessage.replace('.',',')
            self.mqttc.publish("SAC_V5_LeberKaese0001/FPATempCx100", sMQTTMessage)


    def runFfc(self):
        #l.RunRadFfc()
        l.RunSysFFCNormalization()
        self.lastFFCTime = (int(round(time.time() * 1000)))


    def setFluxLinearParams(self):
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
        print("[INFO] FL Params: " + str(FLParams))
        l.SetFluxLinearParams(FLParams)
 
 
    def setThRoiOnLepton(self, thRoi):
        # TH ROI: tuple (start point, end point)
        startPoint, endPoint = thRoi
        l.SetROI((startPoint[0], startPoint[1], endPoint[0], endPoint[1]))
        

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
    
    def getRadTLinearEnableState(self):
        tlStateG = l.GetRadTLinearEnableState()
        print("[INFO] Current TLinear state is: " + str(tlStateG))
        return tlStateG
        
    def setRadTLinearEnableState(self, tlState):
        l.SetRadTLinearEnableState(tlState)
        print("[INFO] Set TLinear state to: " + str(tlState))
        
    def setFfcShutterModeAuto(self, shutterAuto):
        l.SetFfcShutterModeObj(shutterAuto, 0, 1, 0, 0, 180000, 0, 150, 52)
        print("[INFO] Set Auto shutter state to: " + str(shutterAuto))
        
    def getFpaTemp(self):
        fpaTempG = l.GetFpaTemp()
        print("[INFO] Current FPA temperature is: " + str(fpaTempG))
        return fpaTempG        

    def resetStateMachine(self):
        print("[INFO] Resetting state machine...")
        self.state = "IDLE"
        self.temperatureSamples = []
        self.FPATemperatureSamples = []
        self.displayMixer.hide()
        self.lastMeasurementsArePublished = False
        self.NOKRetryCnt = 0 # reset the measurement retry counter
        self.measurementIterCnt = 0 # reset iteration counter


