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
            
        key = ipc.ftok("/home/pi/SACLeptonRPi", ord('l'))
        self.shm = ipc.SharedMemory(key, 0, 0)
        self.shm.attach()

    def show(self, image):     
        # image = 480(h)*640(w)
        start = int(round(time.time()))
        resizeFactor = 1.6875
        image = cv.flip(image, 0)
        print("img size: " + str(image.shape))
        image = cv.resize(image, (1080, 810))
        print("img size: " + str(image.shape))
        r_channel, g_channel, b_channel = cv.split(image)
        alpha_channel = np.ones(b_channel.shape, dtype=b_channel.dtype) * 255 #creating a dummy alpha channel image.
        img_RGBA = cv.merge((r_channel, g_channel, b_channel, alpha_channel))

        reclame = np.zeros([1110, 1080, 4], dtype=np.uint8)
        reclame[:] = (0, 0, 255, 255)
        print("reclame size: " + str(reclame.shape))

        self.shm.write(np.vstack((reclame, img_RGBA)))
        print("Show took: " + str(int(round(time.time())) - start))

    def hide(self):
        transparent = np.zeros([1920, 1080, 4], dtype=np.uint8)
        transparent[:] = (0, 0, 255, 255)
        self.shm.write(transparent)

    def stop(self):
        print("Stopping...")
        self.shm.detach()