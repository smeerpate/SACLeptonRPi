import os
import cv2 as cv
import sysv_ipc as ipc
import time 
import sys
from SACOnScreenDisplay.InputManager import InputManager
from SACOnScreenDisplay.SettingsManager import SettingsManager
from SACOnScreenDisplay.LedDriver import LedDriver
from SACOnScreenDisplay.StateMachine import StateMachine
from SACOnScreenDisplay.OSD import OSD
from SACOnScreenDisplay.DisplayMixer import DisplayMixer
from picamera.array import PiRGBArray
from picamera import PiCamera

displayMixer = DisplayMixer()
settingsManager = SettingsManager()
inputManager = InputManager(5, 6, 13)
ledDriver = LedDriver(17, 27, 22)
f = open("Logging/SAC Temp Log.csv", "a")
stateMachine = StateMachine(settingsManager, ledDriver, f, displayMixer)
osd = OSD(inputManager, settingsManager, displayMixer)

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

camera = PiCamera()
camera.resolution = (640, 480)
rawCapture = PiRGBArray(camera, size=(640, 480))
camera.meter_mode = 'spot'
camera.framerate = 5
time.sleep(0.5)

for data in camera.capture_continuous(rawCapture, format="rgb", use_video_port=True):
    frame = data.array
    frame = frame.copy()
    if osd.isRunning() or inputManager.hasInput():
        stateMachine.reset()
        osd.run(frame)        
    else:
        stateMachine.run(frame)
        gray = cv.cvtColor(frame, cv.COLOR_RGB2GRAY)
        displayMixer.show(gray);

                
    rawCapture.truncate(0)    
    
    

# When everything done, release the capture
ledDriver.stop()
displayMixer.stop()
f.close()
cap.release()
cv.destroyAllWindows()