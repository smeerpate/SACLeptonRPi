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

from Lepton import Lepton
from .LedDriver import LedDriver
from .SettingsManager import SettingsManager
from SACFaceFinder import FaceFinder
from SACForeheadFinder import ForeheadFinder
from .DisplayMixer import DisplayMixer

def calculateTemperature(SensorSamples, FPATempSamples, SkinOffsetTemp, Coeff_A, Coeff_m):
        sensorTemp = sum(SensorSamples) / len(SensorSamples) # average SensorSamples
        fpaTemp = sum(FPATempSamples) / len(FPATempSamples) # average FPATempSamples
        retTemp = sensorTemp + (Coeff_A - fpaTemp * Coeff_m) + SkinOffsetTemp
        print("[INFO] Calculated temperature = " + str(retTemp) + " DegC.")
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
        self.faceSizeUpperLimit = 280
        self.faceSizeLowerLimit = 220
        self.autoTrigger = True # Automatically trigger a measurement every mlAutoTriggerInterval seconds (ignoring Face ROI)
        self.autoTriggerInterval = 20 # seconds 
        self.autoTriggerAtRandomIntervals = True # Requires autoTrigger to be true. Triggers at random intervals
        self.autoTriggerRndIntervalMin = 20 # seconds
        self.autoTriggerRndIntervalMax = 70 # seconds
        self.waitAfterAutotriggerMeas = 3 # seconds, the time to stay in the WAIT FOR NO FACE state in auto trigger mode
        self.measurementInterval = 1 # seconds
        self.measurementIterations = 4 # times, number of measuremnts per head. 
        self.maxFFCInterval = 120 # seconds
        self.maxFPATempInterval = 30 # seconds
        self.showThermalImage = True # Need to show thermal image?
        self.retriesOnResultNOK = 1 #
        self.minTempForValidMeasurement = 31 # minimum temperature for a valid measurement
        self.settleAfterFFCInterval = 0 # seconds to settle after FFC
        self.publishMQTT = True
        self.connectionTimeoutMQTT = 5 # seconds
        self.printTemperatureOnScreen = True
        ##################################
        
        self.lastAutotriggerEvent = 0
        self.lastWaitForNoFaceEntry = 0
        self.lastGetTemperatureEntry = 0
        self.measurementIterCnt = 0
        self.NOKRetryCnt = 0
        self.doSetFLParameters = False
        self.initialParametersAreSet = False
            
        self.temperatureSamples = []
        self.temperature = 0
        self.thSensorWidth = 80
        self.thSensorHeight = 60
        self.FPATemperatureSamples = []
        self.SkinOffset = 0 #2.2 # degrees difference between timpanic temperature and forehead temp
        self.mTempCorrCoeff = 0 # 0.0242 # slope of the linear correction
        self.ATempCorrCoeff = 0 # 1.725 # offset of linear correction
        
        self.lastThermalFrame = []
        self.thColormap = [[255, 255, 255], [253, 253, 253], [251, 251, 251], [249, 249, 249], [247, 247, 247], 
            [245, 245, 245], [243, 243, 243], [241, 241, 241], [239, 239, 239], [237, 237, 237], [235, 235, 235], [233, 233, 233], [231, 231, 231], 
            [229, 229, 229], [227, 227, 227], [225, 225, 225], [223, 223, 223], [221, 221, 221], [219, 219, 219], [217, 217, 217], [215, 215, 215],
            [213, 213, 213], [211, 211, 211], [209, 209, 209], [207, 207, 207], [205, 205, 205], [203, 203, 203], [201, 201, 201], [199, 199, 199],
            [197, 197, 197], [195, 195, 195], [193, 193, 193], [191, 191, 191], [189, 189, 189], [187, 187, 187], [185, 185, 185], [183, 183, 183],
            [181, 181, 181], [179, 179, 179], [177, 177, 177], [175, 175, 175], [173, 173, 173], [171, 171, 171], [169, 169, 169], [167, 167, 167],
            [165, 165, 165], [163, 163, 163], [161, 161, 161], [159, 159, 159], [157, 157, 157], [155, 155, 155], [153, 153, 153], [151, 151, 151],
            [149, 149, 149], [147, 147, 147], [145, 145, 145], [143, 143, 143], [141, 141, 141], [139, 139, 139], [137, 137, 137], [135, 135, 135],
            [133, 133, 133], [131, 131, 131], [129, 129, 129], [126, 126, 126], [124, 124, 124], [122, 122, 122], [120, 120, 120], [118, 118, 118],
            [116, 116, 116], [114, 114, 114], [112, 112, 112], [110, 110, 110], [108, 108, 108], [106, 106, 106], [104, 104, 104], [102, 102, 102],
            [100, 100, 100], [98, 98, 98], [96, 96, 96], [94, 94, 94], [92, 92, 92], [90, 90, 90], [88, 88, 88], [86, 86, 86], [84, 84, 84],
            [82, 82, 82], [80, 80, 80], [78, 78, 78], [76, 76, 76], [74, 74, 74], [72, 72, 72], [70, 70, 70], [68, 68, 68], [66, 66, 66], [64, 64, 64],
            [62, 62, 62], [60, 60, 60], [58, 58, 58], [56, 56, 56], [54, 54, 54], [52, 52, 52], [50, 50, 50], [48, 48, 48], [46, 46, 46], [44, 44, 44],
            [42, 42, 42], [40, 40, 40], [38, 38, 38], [36, 36, 36], [34, 34, 34], [32, 32, 32], [30, 30, 30], [28, 28, 28], [26, 26, 26], [24, 24, 24],
            [22, 22, 22], [20, 20, 20], [18, 18, 18], [16, 16, 16], [14, 14, 14], [12, 12, 12], [10, 10, 10], [8, 8, 8], [6, 6, 6], [4, 4, 4], [2, 2, 2],
            [0, 0, 0], [0, 0, 9], [2, 0, 16], [4, 0, 24], [6, 0, 31], [8, 0, 38], [10, 0, 45], [12, 0, 53], [14, 0, 60], [17, 0, 67], [19, 0, 74], [21, 0, 82],
            [23, 0, 89], [25, 0, 96], [27, 0, 103], [29, 0, 111], [31, 0, 118], [36, 0, 120], [41, 0, 121], [46, 0, 122], [51, 0, 123], [56, 0, 124],
            [61, 0, 125], [66, 0, 126], [71, 0, 127], [76, 1, 128], [81, 1, 129], [86, 1, 130], [91, 1, 131], [96, 1, 132], [101, 1, 133], [106, 1, 134],
            [111, 1, 135], [116, 1, 136], [121, 1, 136], [125, 2, 137], [130, 2, 137], [135, 3, 137], [139, 3, 138], [144, 3, 138], [149, 4, 138],
            [153, 4, 139], [158, 5, 139], [163, 5, 139], [167, 5, 140], [172, 6, 140], [177, 6, 140], [181, 7, 141], [186, 7, 141], [189, 10, 137],
            [191, 13, 132], [194, 16, 127], [196, 19, 121], [198, 22, 116], [200, 25, 111], [203, 28, 106], [205, 31, 101], [207, 34, 95],
            [209, 37, 90], [212, 40, 85], [214, 43, 80], [216, 46, 75], [218, 49, 69], [221, 52, 64], [223, 55, 59], [224, 57, 49], [225, 60, 47],
            [226, 64, 44], [227, 67, 42], [228, 71, 39], [229, 74, 37], [230, 78, 34], [231, 81, 32], [231, 85, 29], [232, 88, 27], [233, 92, 24],
            [234, 95, 22], [235, 99, 19], [236, 102, 17], [237, 106, 14], [238, 109, 12], [239, 112, 12], [240, 116, 12], [240, 119, 12], [241, 123, 12],
            [241, 127, 12], [242, 130, 12], [242, 134, 12], [243, 138, 12], [243, 141, 13], [244, 145, 13], [244, 149, 13], [245, 152, 13], [245, 156, 13],
            [246, 160, 13], [246, 163, 13], [247, 167, 13], [247, 171, 13], [248, 175, 14], [248, 178, 15], [249, 182, 16], [249, 185, 18], [250, 189, 19],
            [250, 192, 20], [251, 196, 21], [251, 199, 22], [252, 203, 23], [252, 206, 24], [253, 210, 25], [253, 213, 27], [254, 217, 28], [254, 220, 29],
            [255, 224, 30], [255, 227, 39], [255, 229, 53], [255, 231, 67], [255, 233, 81], [255, 234, 95], [255, 236, 109], [255, 238, 123], [255, 240, 137],
            [255, 242, 151], [255, 244, 165], [255, 246, 179], [255, 248, 193], [255, 249, 207], [255, 251, 221], [255, 253, 235], [255, 255, 24]]

        self.transformMatrix = self.settingsManager.getSettings().affineTransform.value
        print("[INFO] Loaded affine transform from settings file: " + str(self.transformMatrix))
        
        if self.autoTrigger:
            self.thRoi = (0, 0), (79, 59)
            self.roiFinder = None
        else:
            self.thRoi = (0, 0), (0, 0)
            self.roiFinder = ForeheadFinder()
            self.roiFinder.setTransformMatrix(self.transformMatrix)

        # Globals for logging
        self.currentTime = int(round(time.time()))

        # Globals for FFC timing
        self.lastFFCTime = 0
        self.lastFPATempTime = 0
        
        # mqtt
        if self.publishMQTT:
            mqtt.Client.connected_flag = False # create flag in class
            self.mqttBrokerAddress = "192.168.1.60" #"192.168.0.138" # "192.168.1.60"
            self.mqttBrokerPort = 1883
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
    ###########################################
    def addThermalImage(self, image):
        # Capture image from Lepton.
        #thRaw,_ = self.lepton.capture()
        if len(self.lastThermalFrame) == self.thSensorHeight * self.thSensorWidth:
            thRaw = np.reshape(self.lastThermalFrame, (self.thSensorHeight, self.thSensorWidth))
            cv.normalize(thRaw, thRaw, 0, 65535, cv.NORM_MINMAX) # extend contrast
            np.right_shift(thRaw, 8, thRaw) # fit raw data into 8 bit
            # make uint8 image
            thermal = np.uint8(thRaw)
        else:
            thermal = np.full((self.thSensorHeight, self.thSensorWidth), 100, dtype=np.uint8)
        # convert grayscale to BGR
        #thermal = cv.cvtColor(thermal, cv.COLOR_GRAY2BGR)
        thermal = cv.applyColorMap(thermal, cv.COLORMAP_HOT)
        #thermal = cv.LUT(thermal, self.thColormap)
        xOffset = yOffset = 10
        #print(image.shape)
        #print(thermal.shape)
        #image[yOffset:yOffset+thermal.shape[0], xOffset:xOffset+thermal.shape[1],:] = thermal # TODO: make offset adjustable and use width and height from Lepton library
        thermal = cv.resize(thermal, (640-xOffset, 480-yOffset))
        return thermal

    def addRectangle(self, image, roi, color):
        # ROI: tuple (start point, end point)
        startPoint, endPoint = roi
        cv.rectangle(image, startPoint, endPoint, color, 1)

    def writeLog(self):
        line = str(int(round(time.time()))) + ";" + str(self.roiFinder.name) + ";" + str(l.GetAuxTemp()) + ";" + str(l.GetFpaTemp()) + ";" + str(l.GetROIValues()) + ";" + str(l.GetROI()) + ";" + str(self.thRoi) + ";" + str(int(round(self.lastFFCTime))) + "\n"
        print("SAC_TEMPLOG: " + line)
        #self.logFile.write(line)

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
        # save last state
        self.prevState = self.state
        try:
            # Update settings
            settings = self.settingsManager.getSettings()
            # Get current time
            smEntryTimeMs = int(round(time.time() * 1000))
            
            # Steps:
            # 1) Find a face
            # 2) Check if face size is good
            # 3) If face size == good -> calibration of Lepton (FFC) + measure amb temp (currently this will be done via the lepton, later on we will use an external temp sensor)
            # 4) Get Affine coords + set ROI
            # 5) Get RIO data (can be repeated x amount of times to be sure)
            # 6) If temp < threshold -> ok
            # 7) Else -> inform the user that we are going to measure again

            if self.state == "IDLE":
                color = settings.idleColor
                self.ledDriver.output(color.red, color.green, color.blue, 100)
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
                else:
                    if self.roiFinder.getTcContours(image, settings.showFoundFace.value):
                        self.state = "WAIT_FOR_SIZE_OK"
                        self.displayMixer.showDontMove(image)
                        dateValue = datetime.datetime.fromtimestamp(time.time())
                        print("[INFO] Found a face. (Timestamp = " + dateValue.strftime('%Y-%m-%d %H:%M:%S') + ")")
                    else:
                       # if self.roiFinder.faceFound:
                       #     self.displayMixer.showDontMove(image)
                       # else:
                       #     self.displayMixer.hide()
                        self.displayMixer.hide()
                        # did we already set the initial parameters?
                        if self.initialParametersAreSet:
                            self.state = "IDLE"
                        else:
                            self.state = "SET_INITIAL_PARAMETERS"
                            
                        self.currentTime = int(round(time.time() * 1000))
                        if self.currentTime > (self.lastFPATempTime + (self.maxFPATempInterval * 1000)):
                            self.getFpaTemp()
                            self.lastFPATempTime = self.currentTime
                    #print("Idle took: " + str(int(round(time.time() * 1000)) - smEntryTimeMs) + "ms")

     
            elif self.state == "SET_INITIAL_PARAMETERS":
                self.setFluxLinearParams()
                self.setRadTLinearEnableState(1)
                self.setFfcShutterModeAuto(0)
                self.initialParametersAreSet = True
                print("[INFO] Initial parametars are set.")
                self.state = "IDLE"
                
                
            elif self.state == "WAIT_FOR_SIZE_OK":
                #start = int(round(time.time() * 1000))
                if self.roiFinder.getTcContours(image, settings.showFoundFace.value):
                    if not self.checkFaceSize(image, self.roiFinder.getTcROIWidth(), self.faceSizeLowerLimit, self.faceSizeUpperLimit):
                        self.state = "WAIT_FOR_SIZE_OK"
                        self.displayMixer.showDontMove(image)
                    else:
                        self.currentTime = int(round(time.time()))
                        if self.currentTime > (self.lastFFCTime + self.maxFFCInterval):
                            self.state = "RUN_FFC"                        
                        else:
                            self.state = "SET_FLUX_LINEAR_PARAMS"
                        self.displayMixer.showDontMove(image)
                else:
                    self.state = "IDLE"
                    self.temperatureSamples = []
                    self.FPATemperatureSamples = []
                    self.displayMixer.hide()
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
                self.NOKRetryCnt = 0 # reset the measurement retry counter
                self.measurementIterCnt = 0 # reset iteration counter
                self.doSetFLParameters = False # only set Flux liear parameters once
                
            
            elif self.state == "SETTLTE_AFTER_FFC":
                if smEntryTimeMs > (self.lastFFCTime + (self.settleAfterFFCInterval * 1000)):
                    self.state = "GET_TEMPERATURE"
                else:
                    self.state = "SETTLTE_AFTER_FFC"


            elif self.state == "GET_TEMPERATURE":
                if self.autoTrigger:
                    self.temperatureSamples.append(self.measureTemp())
                    self.FPATemperatureSamples.append(self.getFpaTemp())
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

                        thRoi = self.getRoiFromContours(thRoiContours)

                        # Flip thRoi vertically
                        #start, end = thRoi
                        #w = end[0] - start[0]
                        #thRoi = (80 - start[0] - w, start[1]), (80 - start[0], end[1])

                        # Translate a bit to the right
                        start, end = thRoi
                        xTrans = 10
                        thRoi = (start[0] + xTrans, start[1]), (end[0] + xTrans, end[1])

                        self.setThRoiOnLepton(thRoi)
                        #self.writeLog()
                        values = l.GetROIValues()
                        self.temperatureSamples.append(values[1])
                        self.FPATemperatureSamples.append(self.getFpaTemp())
                    else:
                        self.state = "IDLE"
                        self.temperatureSamples = []
                        self.FPATemperatureSamples = []
                        self.displayMixer.hide()
                #print("Get temp took: " + str(int(round(time.time() * 1000)) - startTime) + "ms")
                
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
                    
            
            
            elif self.state == "WAIT_TO_TAKE_NEXT_SAMPLE":
                if smEntryTimeMs > (self.lastGetTemperatureEntry + (self.measurementInterval * 1000)):
                    self.state = "GET_TEMPERATURE"
                else:
                    self.state = "WAIT_TO_TAKE_NEXT_SAMPLE"

                
            elif self.state == "EVALUATE_RESULT":
                self.temperature = calculateTemperature(self.temperatureSamples, self.FPATemperatureSamples, self.SkinOffset, self.ATempCorrCoeff, self.mTempCorrCoeff)
                if self.showThermalImage:
                    image = self.addThermalImage(image)
                if self.printTemperatureOnScreen:
                    self.addText(image, str(self.temperature), (0,255,255))
                if self.temperature > settings.threshold.value:
                    if self.NOKRetryCnt < self.retriesOnResultNOK:
                        # OK, bad measurement: retry measuring
                        print("[INFO] Temperature was over threshold (" + str(self.temperature) + "DegC), retrying.")
                        self.measurementIterCnt = 0 # reset iteration counter
                        self.state = "GET_TEMPERATURE"
                        self.NOKRetryCnt = self.NOKRetryCnt + 1
                    else:
                        color = settings.alarmColor
                        self.displayMixer.showTemperatureNok(image)
                        self.state = "WAIT_FOR_NO_FACE"                  
                elif self.temperature < self.minTempForValidMeasurement:
                    if self.NOKRetryCnt < self.retriesOnResultNOK:
                        # OK, bad measurement: retry measuring
                        print("[INFO] Measured temperature (" + str(self.temperature) + "DegC) was not valid, retrying.")
                        self.measurementIterCnt = 0 # reset iteration counter
                        self.state = "GET_TEMPERATURE"
                        self.NOKRetryCnt = self.NOKRetryCnt + 1
                    else:
                        color = settings.alarmColor
                        self.displayMixer.showTemperatureNok(image)
                        self.state = "WAIT_FOR_NO_FACE"
                else:
                    color = settings.okColor
                    self.displayMixer.showTemperatureOk(image)
                    self.state = "WAIT_FOR_NO_FACE"
                
                

            elif self.state == "WAIT_FOR_NO_FACE":
                self.publishMeasurements()
                if self.autoTrigger:
                    time.sleep(self.waitAfterAutotriggerMeas)
                    self.state = "IDLE"
                    self.temperatureSamples = []
                    self.displayMixer.hide()
                    if self.autoTriggerAtRandomIntervals:
                        self.autoTriggerInterval = random.randint(self.autoTriggerRndIntervalMin, self.autoTriggerRndIntervalMax)
                        print("[INFO] Random intervals set, next measurement in " + str(self.autoTriggerInterval) + " seconds.")
                else:
                    self.roiFinder.getTcContours(image, False) # only looks for the head now
                    if self.roiFinder.faceFound:
                        print("[INFO] Max temp of all measurements: " + str(self.temperature) + " DegC" + ". (" + ', '.join(str(e) for e in self.temperatureSamples) + ")")    

                        if self.temperature > settings.threshold.value:
                            self.displayMixer.showTemperatureNok(image)
                        else:
                            self.displayMixer.showTemperatureOk(image)                       
                    else:
                        self.state = "IDLE"
                        self.temperatureSamples = []
                        self.displayMixer.hide()
        except Exception as e:
            print("[ERROR] " + e.message)
        

    def measureTemp(self):
        print("[INFO] Reading framebuffer...")
        thFrame = l.GetFrameBuffer("/dev/spidev0.0")
        if type(thFrame) is str:
            # GetFrameBuffer returns a string on error
            print("[ERROR] " + thFrame)
            return 0
        else:
            self.lastThermalFrame = np.array(thFrame)
            return (max(thFrame)/100) - 273.15
        
    def publishMeasurements(self):
        if self.publishMQTT:
            # Get FPA and AUX temp an put it in a csv line like this:
            # "Current time;AUX Temp;FPA Temp;Last FFC time;"
            sMQTTMessage = str(int(round(time.time() * 1000))) + ';' + str(l.GetAuxTemp()) + ';' + str(l.GetFpaTemp()) + ';' + str(self.lastFFCTime) + ';' + str(self.temperature) + ";"
            sMQTTMessage = sMQTTMessage.replace('.',',')
            self.mqttc.publish("TempCx100", sMQTTMessage)
            thFrame = l.GetFrameBuffer("/dev/spidev0.0")
            if 0:
                print("[INFO] Framebuffer length is: " + str(len(thFrame)))
            # publish the frame
            self.mqttc.publish("FrameBuffer", struct.pack('%sH' %len(thFrame), *thFrame))
        else:
            pass


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
        #print("Setting ROI on Lepton:")
        #print(str(thRoi))

        startPoint, endPoint = thRoi

        #print(str(l.SetROI((startPoint[0], startPoint[1], endPoint[0], endPoint[1]))))
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
        
    

    def showThermalImage(self):
        #raw,_ = self.lepton.capture()
        #cv.normalize(raw, raw, 0, 65535, cv.NORM_MINMAX)
        #np.right_shift(raw, 8, raw)
        #thImage = np.uint8(raw) # 80x60

        #self.addRectangle(thImage, thRoi, (255, 255, 255))
        #self.addRectangle(thImage, thCorrected, (255, 0, 0))
        #self.roiFinder.getTcContours(image, settings.showFoundFace.value)
        #x_offset=y_offset=0
        #image[y_offset:y_offset+thImage.shape[0], x_offset:x_offset+thImage.shape[1]] = thImage
        #imageName = "Forehead " + str(int(round(time.time())))
        #cv.imwrite('/home/pi/SACLeptonRPi/' + imageName +'.jpg', image)
        return

    def reset(self):
        print("[INFO] Resetting state machine...")
        self.state = "IDLE"


