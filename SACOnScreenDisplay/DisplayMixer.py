from subprocess import call
from threading import Thread
import time
import sysv_ipc as ipc
import cv2 as cv

def startDisplay():
        call(["./SACDisplayMixer/OGLESSimpleImageWithIPC"])

class DisplayMixer(object):
    """description of class"""

    def __init__(self):
        self.isRunning = False;
        self.shm = None;
        self.mixerThread = None

    def show(self, image):       
        if not self.isRunning:
            #self.mixerThread = Thread(target=startDisplay)
            #self.mixerThread.start()
            #time.sleep(1)
            
            key = ipc.ftok("/home/pi/SACLeptonRPi", ord('i'))
            self.shm = ipc.SharedMemory(key, 0, 0)
            self.shm.attach()
            self.isRunning = True;

        image = cv.flip(image, 0)
        self.shm.write(image)
        startDisplay()

    def stop(self):
        if self.isRunning:
            print("Stopping...")
            self.shm.detach()
            self.isRunning = False;