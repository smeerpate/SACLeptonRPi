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
        
        self.screenWidth = 1920
        self.screenHeight = 1080

        self.measuring = cv.imread("/home/pi/windmaster/Slides/SAC_MEASURING.jpg") 
        self.measuring = cv.cvtColor(self.measuring, cv.COLOR_BGR2RGBA)
        self.measuring = cv.flip(self.measuring, 0)

        self.tempOk = cv.imread("/home/pi/windmaster/Slides/SAC_TEMPOK.jpg")
        self.tempOk = cv.cvtColor(self.tempOk, cv.COLOR_BGR2RGBA)
        self.tempOk = cv.flip(self.tempOk, 0)

        self.tempNok = cv.imread("/home/pi/windmaster/Slides/SAC_TEMPNOK.jpg")
        self.tempNok = cv.cvtColor(self.tempNok, cv.COLOR_BGR2RGBA)
        self.tempNok = cv.flip(self.tempNok, 0)

        self.dontMove = cv.imread("/home/pi/windmaster/Slides/SAC_DONTMOVE.jpg")
        self.dontMove = cv.cvtColor(self.dontMove, cv.COLOR_BGR2RGBA)
        self.dontMove = cv.flip(self.dontMove, 0)
        
        self.settling = cv.imread("/home/pi/windmaster/Slides/SAC_SETTLING.jpg")
        self.settling = cv.cvtColor(self.settling, cv.COLOR_BGR2RGBA)
        self.settling = cv.flip(self.settling, 0)
        
        self.stepCloser = cv.imread("/home/pi/windmaster/Slides/SAC_STEPCLOSER.jpg")
        self.stepCloser = cv.cvtColor(self.stepCloser, cv.COLOR_BGR2RGBA)
        self.stepCloser = cv.flip(self.stepCloser, 0)
        
        self.stepBack = cv.imread("/home/pi/windmaster/Slides/SAC_STEPBACK.jpg")
        self.stepBack = cv.cvtColor(self.stepBack, cv.COLOR_BGR2RGBA)
        self.stepBack = cv.flip(self.stepBack, 0)
        #Display is 1920 (width) x 1080 (height) -> np array is 1080 (rows) x 1920 (cols)
        #self.transparent = np.full([1080, 200, 4], 150, dtype=np.uint8)
        #self.transparent = np.hstack((self.transparent, np.full([1080, 1720, 4], 240, dtype=np.uint8)))
        self.transparent = np.full([1080, 1920, 4], 0, dtype=np.uint8)

    def show(self, image, slide):     
        # image = 480(h)*640(w)
        #image = cv.flip(image, -1) # flip vertically and mirror
        image = cv.flip(image, 0) # flip vertically
        origImageWidth = image.shape[1]
        origImageHeight = image.shape[0]
        # we're going portrait and keeping the aspect ratio of the image
        scaleFactor = self.screenHeight / origImageWidth
        newImageWidth = self.screenHeight
        newImageHeight = int(origImageHeight * scaleFactor)
        # apply resize and rotate
        image = cv.resize(image, (newImageWidth, newImageHeight))
        image = cv.rotate(image, cv.ROTATE_90_CLOCKWISE)
        #print("[INFO] New true color image size: " + str(image.shape))
        
        b_channel, g_channel, r_channel = cv.split(image)
        alpha_channel = np.ones(b_channel.shape, dtype=b_channel.dtype) * 255 #creating a dummy alpha channel image.
        img_RGBA = cv.merge((r_channel, g_channel, b_channel, alpha_channel))

        slide = cv.resize(slide, (self.screenWidth-newImageHeight, self.screenHeight))
        #print("[INFO] New slide size: " + str(slide.shape))
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
        
    def showSettling(self, image):
        self.show(image, self.settling)
        
    def showStepBack(self, image):
        self.show(image, self.stepBack)
        
    def showStepCloser(self, image):
        self.show(image, self.stepCloser)

    def hide(self):
        start = int(round(time.time() * 1000))
        self.shm.write(self.transparent)
        #print("writing transparent: " + str(int(round(time.time() * 1000)) - start) + "ms")

    def stop(self):
        print("Stopping...")
        self.shm.detach()