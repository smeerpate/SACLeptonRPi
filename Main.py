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

try:
    displayMixer = DisplayMixer()
    settingsManager = SettingsManager()
    inputManager = InputManager(5, 6, 13)
    ledDriver = LedDriver(17, 27, 22)
    #f = open("/home/pi/SACLeptonRPi/Logging/SAC Temp Log.csv", "a")
    stateMachine = StateMachine(settingsManager, ledDriver, displayMixer)
    osd = OSD(inputManager, settingsManager, displayMixer)

    camera = PiCamera()
    camera.resolution = (640, 480)
    rawCapture = PiRGBArray(camera, size=(640, 480))
    camera.meter_mode = 'spot'
    camera.framerate = 30
    time.sleep(0.5)

    for data in camera.capture_continuous(rawCapture, format="rgb", use_video_port=True):
        frame = data.array
        #frame = frame.copy()
        #if osd.isRunning() or inputManager.hasInput():
        #    stateMachine.reset()
        #    osd.run(frame)        
        #else:
        #    stateMachine.run(frame) 
        rawCapture.truncate()
        rawCapture.seek(0)
        stateMachine.run(frame)

    # When everything done, release the capture
    ledDriver.stop()
    displayMixer.stop()
    f.close()
    cap.release()
    cv.destroyAllWindows()
    
except Exception as Argument:
    # When everything done, release the capture
    print(str(Argument))
    ledDriver.stop()
    displayMixer.stop()
    f.close()
    cap.release()
    cv.destroyAllWindows()