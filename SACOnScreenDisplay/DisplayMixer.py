from subprocess import call
from threading import Thread
import time
import sysv_ipc as ipc
import cv2 as cv
import numpy as np

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
            self.mixerThread = Thread(target=startDisplay)
            self.mixerThread.start()
            time.sleep(1)
            
            key = ipc.ftok("/home/pi/SACLeptonRPi", ord('k'))
            self.shm = ipc.SharedMemory(key, 0, 0)
            self.shm.attach()
            self.isRunning = True;

        image = cv.flip(image, 0)
        b_channel, g_channel, r_channel = cv.split(image)
        alpha_channel = np.ones(b_channel.shape, dtype=b_channel.dtype) * 255 #creating a dummy alpha channel image.
        img_RGBA = cv.merge((r_channel, g_channel, b_channel, alpha_channel))
        self.shm.write(img_RGBA);
        #self.shm.write(np.zeros([480, 640, 4], dtype=np.uint8))
        #startDisplay()

    def hide(self):
        transparent = np.zeros([480, 640, 4], dtype=np.uint8)
        self.shm.write(transparent)

    def stop(self):
        if self.isRunning:
            print("Stopping...")
            self.shm.detach()
            self.isRunning = False;