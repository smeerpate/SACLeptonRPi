import numpy as np
import sysv_ipc as ipc
import time
from subprocess import call
from threading import Thread

def startDisplay():
	call(["./SACCreateEGLWindowWithIPC"])


th1 = Thread(target=startDisplay)
th1.start()
time.sleep(1)

key = ipc.ftok(".", ord('s'))
shm = ipc.SharedMemory(key, 0, 0)

imContent = np.ones((640,480,4),dtype=np.uint8)

shm.attach()

for cnt in range(255):
	imRamp = imContent * cnt
	shm.write(imRamp)
	print("matrix values are all set to " + str(cnt))
	time.sleep(0.5)

shm.detach()
