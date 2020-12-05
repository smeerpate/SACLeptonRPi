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
            self.shm = ipc.SharedMemory(key, 0, 0)
            self.shm.attach()
            self.isRunning = True;

        self.shm.write(image)

    def stop(self):
        if self.isRunning:
            self.shm.detach()
            self.isRunning = False;