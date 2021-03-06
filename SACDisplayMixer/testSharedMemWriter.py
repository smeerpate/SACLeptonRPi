import numpy as np
import sysv_ipc as ipc
import time
from subprocess import call
from threading import Thread
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera

def startDisplay():
	call(["./OGLESSimpleImageWithIPC"])


th1 = Thread(target=startDisplay)
th1.start()
time.sleep(1)

key = ipc.ftok("/home/pi/SACLeptonRPi", ord('i'))
shm = ipc.SharedMemory(key, 0, 0)

#imContent = np.ones((640,480,3),dtype=np.uint8)
camera = PiCamera()
camera.resolution = (640, 480)
rawCapture = PiRGBArray(camera, size=(640, 480))
# give time to startup the camera...
time.sleep(0.1)

shm.attach()

# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="rgb", use_video_port=True):
	# grab the raw NumPy array representing the image, then initialize the timestamp
	# and occupied/unoccupied text
	image = frame.array
	# write the frame to shared memory
	shm.write(image)

	if cv2.waitKey(0) & 0xFF == ord('q'):
		break
	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)

	time.sleep(0.1)


shm.detach()
