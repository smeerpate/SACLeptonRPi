import cv2 as cv
import time
from picamera.array import PiRGBArray
from picamera import PiCamera
print(cv.__file__)

net = cv.dnn.readNet('face-detection-adas-0001.xml', 'face-detection-adas-0001.bin')
print("Read net completed")

net.setPreferableTarget(cv.dnn.DNN_TARGET_MYRIAD)
print("Target set")

camera = PiCamera()
camera.resolution = (640, 480)
rawCapture = PiRGBArray(camera, size=(640, 480))
camera.meter_mode = 'spot'
camera.framerate = 30
#frame = cv.imread('test.jpg')
#print("image read")
#cv.imshow('input', frame)
#cv.waitKey(0)
time.sleep(0.5)

for data in camera.capture_continuous(rawCapture, format="rgb", use_video_port=True):
	frame = data.array
	start = time.time()
	blob = cv.dnn.blobFromImage(frame, size=(640,480), ddepth=cv.CV_8U)
	print("blob created from image")
	net.setInput(blob)
	print("set input")
	out = net.forward()
	print("forwarded")

	for detection in out.reshape(-1, 7):
		confidence = float(detection[2])
		if confidence > 0.5:
			xmin = int(detection[3] * frame.shape[1])
			ymin = int(detection[4] * frame.shape[0])
			xmax = int(detection[5] * frame.shape[1])
			ymax = int(detection[6] * frame.shape[0])
		
			#print("xmin: " + str(xmin))
			cv.rectangle(frame, (xmin, ymin), (xmax, ymax), color=(255,0,0))
		cv.putText(frame, "Confidence: " + str(confidence), (50, 50), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1)
	timespan = (time.time() - start) * 1000
	print("Time to detect (all-in)(ms): " + str(timespan))
	cv.imshow('Image', frame)
	cv.waitKey(20)
	rawCapture.truncate(0)
cv.waitKey(0)
cv.destroyAllWindows()
#cv.imwrite('out.png', frame)
