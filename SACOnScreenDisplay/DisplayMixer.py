from subprocess import call
from threading import Thread
import time
import sysv_ipc as ipc

def startDisplay():
        call(["./SACDisplayMixer/OGLESSimpleImageWithIPC"])

class DisplayMixer(object):
    """description of class"""

    def __init__(self):
        self.isRunning = False;
        self.shm = None;

    def show(self, image):       
        if not self.isRunning:
            th1 = Thread(target=startDisplay)
            th1.start()
            time.sleep(1)

            key = ipc.ftok("/home/pi/SACLeptonRPi", ord('i'))
            shm = ipc.SharedMemory(key, 0, 0)
            shm.attach()
            self.isRunning = True;

        shm.write(image)

    def stop(self):
        if self.isRunning:
            shm.detach()
            self.isRunning = False;