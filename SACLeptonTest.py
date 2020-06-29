import numpy as np
import cv2
import time
from Lepton import Lepton
import sys

# Stop the cursor from blinking
sys.stdout.write("\033[?25l")
sys.stdout.flush()

# Target screen is 12", 1024x768
screenWidth = 1024
screenHeight = 768
# Sensor size is 80x60
sensorWidth = 80
sensorHeight = 60
# Lepton offset in degC
corrVal = 1.45

# Initialize Lepton sensor instance.
l = Lepton()

# create full screen output window
#cv2.namedWindow("output", cv2.WINDOW_NORMAL)
#cv2.setWindowProperty("output",cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

# make an alpha channel for the frame buffer image.
alpha = np.ones((screenHeight,screenWidth), dtype=np.uint8)*255



while(1):
    raw,_ = l.capture()
    # find maximum value in raw array
    maxVal = np.amax(raw)
    maxCoord = np.where(raw == maxVal)
#    print(maxCoord)
#    print(maxCoord[0][0])
#    print(maxCoord[1][0])
    # text position
    txtPosition = (maxCoord[1][0]*screenWidth/sensorWidth, maxCoord[0][0]*screenHeight/sensorHeight)
#    print(txtPosition)

    cv2.normalize(raw, raw, 0, 65535, cv2.NORM_MINMAX) # extend contrast
    np.right_shift(raw, 8, raw) # fit data into 8 bits
    # scale the image to full screen resolution.
    resized = cv2.resize(np.uint8(raw), (screenWidth ,screenHeight), interpolation = cv2.INTER_AREA)
    # convert grayscale to BGR
    color = cv2.cvtColor(resized, cv2.COLOR_GRAY2BGR)
    
    # Put interesting data on top of the image.
    cv2.putText(color, "{}".format((maxVal/100)-273.15 + corrVal), txtPosition, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,150,0,255), 2)
    #cv2.imshow("output", color) # show it.
    b,g,r = cv2.split(color)
    fbImage = cv2.merge((b,g,r,alpha))
    with open('/dev/fb0', 'rb+') as fBuf:
        fBuf.write(fbImage)
    
    # break als de esc knop ingedrukt wordt
    if cv2.waitKey(20) & 0xFF == 27:
        break
    time.sleep(0.1)
