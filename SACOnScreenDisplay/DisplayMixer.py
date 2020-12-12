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
        self.mixerThread = Thread(target=startDisplay)
        self.mixerThread.start()
        time.sleep(1)
            
        key = ipc.ftok("/home/pi/SACLeptonRPi", ord('k'))
        self.shm = ipc.SharedMemory(key, 0, 0)
        self.shm.attach()

    def show(self, image):     
        image = cv.flip(image, 0)
        r_channel, g_channel, b_channel = cv.split(image)
        alpha_channel = np.ones(b_channel.shape, dtype=b_channel.dtype) * 255 #creating a dummy alpha channel image.
        img_RGBA = cv.merge((r_channel, g_channel, b_channel, alpha_channel))
        self.shm.write(img_RGBA);

    def hide(self):
        transparent = np.zeros([1920, 1080, 4], dtype=np.uint8)
        transparent[:] = (0, 0, 255, 255)
        self.shm.write(transparent)

    def stop(self):
        print("Stopping...")
        self.shm.detach()