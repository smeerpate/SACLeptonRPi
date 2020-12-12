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

    def show(self, image, slideName):     
        # image = 480(h)*640(w)
        #start = int(round(time.time() * 1000))
        resizeFactor = 1.6875
        image = cv.flip(image, 0)
        print("img size: " + str(image.shape))
        image = cv.resize(image, (1080, 810))
        print("img size: " + str(image.shape))
        r_channel, g_channel, b_channel = cv.split(image)
        alpha_channel = np.ones(b_channel.shape, dtype=b_channel.dtype) * 255 #creating a dummy alpha channel image.
        img_RGBA = cv.merge((r_channel, g_channel, b_channel, alpha_channel))

        start = int(round(time.time() * 1000))
        slide = cv.imread(slideName)
        slide = cv.cvtColor(slide, cv.COLOR_RGB2RGBA)        
        slide = cv.flip(slide, 0) # maybe flipped on disk instead of doing it codewise???? because these slides are hardcoded so...
        print("read, cvt + flip took: " + str(int(round(time.time() * 1000)) - start) + "ms")
        #reclame = np.zeros([1110, 1080, 4], dtype=np.uint8)
        #reclame[:] = (0, 0, 255, 255)
        print("slide size: " + str(slide.shape))
        start = int(round(time.time() * 1000))
        self.shm.write(np.vstack((slide, img_RGBA)))
        print("Show took: " + str(int(round(time.time() * 1000)) - start) + "ms")

    def showMeasuring(self, image):
        self.show(image, "Slides/SAC_MEASURING.jpg")

    def showTemperatureOk(self, image):
        self.show(image, "Slides/SAC_TEMPOK.jpg")

    def showTemperatureNok(self, image):
        self.show(image, "Slides/SAC_TEMPNOK.jpg")

    def showDontMove(self, image):
        self.show(image, "Slides/SAC_DONTMOVE.jpg")

    def hide(self):
        start = int(round(time.time() * 1000))
        transparent = np.full([1920, 1080, 4], 0, dtype=np.uint8)
        print("Creating transparent: " + str(int(round(time.time() * 1000)) - start) + "ms")
        start = int(round(time.time() * 1000))
        self.shm.write(transparent)
        print("writing transparent: " + str(int(round(time.time() * 1000)) - start) + "ms")

    def stop(self):
        print("Stopping...")
        self.shm.detach()