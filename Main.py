import os
import cv2 as cv
import sysv_ipc as ipc
import time 
import sys
from subprocess import call
from threading import Thread
from SACOnScreenDisplay.InputManager import InputManager
from SACOnScreenDisplay.SettingsManager import SettingsManager
from SACOnScreenDisplay.LedDriver import LedDriver
from SACOnScreenDisplay.StateMachine import StateMachine
from SACOnScreenDisplay.OSD import OSD
from picamera.array import PiRGBArray
from picamera import PiCamera

settingsManager = SettingsManager()
inputManager = InputManager(5, 6, 13)
ledDriver = LedDriver(17, 27, 22)
f = open("SAC Temp Log.csv", "a")
stateMachine = StateMachine(settingsManager, ledDriver, f)
osd = OSD(inputManager, settingsManager)

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

def startDisplay():
    call(["./SACDisplayMixer/OGLESSimpleImageWithIPC"])
   
th1 = Thread(target=startDisplay)
th1.start()
time.sleep(1)

key = ipc.ftok("/home/pi/SACLeptonRPi", ord('i'))
shm = ipc.SharedMemory(key, 0, 0)

camera = PiCamera()
camera.resolution = (640, 480)
rawCapture = PiRGBArray(camera, size=(640, 480))
camera.meter_mode = 'spot'
camera.framerate = 5
time.sleep(0.5)

shm.attach()

for data in camera.capture_continuous(rawCapture, format="rgb", use_video_port=True):
    frame = data.array
    frame = frame.copy()
    if osd.isRunning() or inputManager.hasInput():
        stateMachine.reset()
        osd.run(frame)        
    else:
        stateMachine.run(frame)
                
    rawCapture.truncate(0)
    frame = cv.flip(frame, 0)
    
    #print(frame.shape)
    
    shm.write(frame)

    #if stateMachine.state == "WAIT_FOR_NO_FACE":
        #time.sleep(5)
    
    

# When everything done, release the capture
ledDriver.stop()
shm.detach()
f.close()
cap.release()
cv.destroyAllWindows()