import numpy as np
import cv2
from Lepton import Lepton

l = Lepton()
a,_ = l.capture()
cv2.normalize(a, a, 0, 65535, cv2.NORM_MINMAX) # extend contrast
np.right_shift(a, 8, a) # fit data into 8 bits

cv2.namedWindow("output", cv2.WINDOW_NORMAL)
cv2.setWindowProperty("output",cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

while(1):
    cv2.imshow("output", np.uint8(a)) # show it.
    # break als de esc knop ingedrukt wordt
    if cv2.waitKey(20) & 0xFF == 27:
        break

