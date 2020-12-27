from subprocess import call
from threading import Thread
import time
import sysv_ipc as ipc
import cv2 as cv
import numpy as np

def startDisplay():
        call(["/home/pi/SACLeptonRPi/SACDisplayMixer/OGLESSimpleImageWithIPC"])

class DisplayMixer(object):
    """description of class"""

    def __init__(self):
        self.mixerThread = Thread(target=startDisplay)
        self.mixerThread.start()
        time.sleep(1)
            
        key = ipc.ftok("/home/pi/SACLeptonRPi", ord('p'))
        self.shm = ipc.SharedMemory(key, 0, 0)
        self.shm.attach()

        self.measuring = cv.imread("/home/pi/windmaster/Slides/SAC_MEASURING.jpg")
        self.measuring = cv.cvtColor(self.measuring, cv.COLOR_RGB2RGBA)
        self.measuring = cv.flip(self.measuring, 0)

        self.tempOk = cv.imread("/home/pi/windmaster/Slides/SAC_TEMPOK.jpg")
        self.tempOk = cv.cvtColor(self.tempOk, cv.COLOR_RGB2RGBA)
        self.tempOk = cv.flip(self.tempOk, 0)

        self.tempNok = cv.imread("/home/pi/windmaster/Slides/SAC_TEMPNOK.jpg")
        self.tempNok = cv.cvtColor(self.tempNok, cv.COLOR_RGB2RGBA)
        self.tempNok = cv.flip(self.tempNok, 0)

        self.dontMove = cv.imread("/home/pi/windmaster/Slides/SAC_DONTMOVE.jpg")
        self.dontMove = cv.cvtColor(self.dontMove, cv.COLOR_RGB2RGBA)
        self.dontMove = cv.flip(self.dontMove, 0)
        #Display is 1920 (width) x 1080 (height) -> np array is 1080 (rows) x 1920 (cols)
        #self.transparent = np.full([1080, 200, 4], 150, dtype=np.uint8)
        #self.transparent = np.hstack((self.transparent, np.full([1080, 1720, 4], 240, dtype=np.uint8)))
        self.transparent = np.full([1080, 1920, 4], 0, dtype=np.uint8)

    def show(self, image, slide):     
        # image = 480(h)*640(w)
        #start = int(round(time.time() * 1000))
        resizeFactor = 1.6875
        image = cv.flip(image, 0)        
        #print("img size: " + str(image.shape))
        image = cv.resize(image, (1080, 810))
        image = cv.rotate(image, cv.ROTATE_90_CLOCKWISE)
        #print("img size: " + str(image.shape))
        r_channel, g_channel, b_channel = cv.split(image)
        alpha_channel = np.ones(b_channel.shape, dtype=b_channel.dtype) * 255 #creating a dummy alpha channel image.
        img_RGBA = cv.merge((r_channel, g_channel, b_channel, alpha_channel))

        #print("slide size: " + str(slide.shape))
        #slide = np.full([1080, 1110, 4], 200, dtype=np.uint8)
        start = int(round(time.time() * 1000))
        self.shm.write(np.hstack((img_RGBA, slide)))
        #print("Show took: " + str(int(round(time.time() * 1000)) - start) + "ms")

    def showMeasuring(self, image):
        self.show(image, self.measuring)

    def showTemperatureOk(self, image):
        self.show(image, self.tempOk)

    def showTemperatureNok(self, image):
        self.show(image, self.tempNok)

    def showDontMove(self, image):
        self.show(image, self.dontMove)

    def hide(self):
        start = int(round(time.time() * 1000))
        self.shm.write(self.transparent)
        #print("writing transparent: " + str(int(round(time.time() * 1000)) - start) + "ms")

    def stop(self):
        print("Stopping...")
        self.shm.detach()