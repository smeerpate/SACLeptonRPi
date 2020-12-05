from .Setting import Setting
from .ButtonInput import ButtonInput
import cv2 as cv
import numpy as np

class AffineTransformSetting(Setting):
    """description of class"""

    def __init__(self, name):
        super().__init__(name)
        self.value = np.array([[1.70100612e-1, 4.91086300e-4, -2.62737066e+1],[5.51191729e-3, 1.75597084e-1, -2.09686199e+1]]) #Default

    def show(self, image):
        color = (255, 0, 0)
        row1 = str(self.value[0][0]) + ", " + str(self.value[0][1]) + ", " + str(self.value[0][2])
        row2 = str(self.value[1][0]) + ", " + str(self.value[1][1]) + ", " + str(self.value[1][2])
        cv.putText(image, row1, (0,50) , cv.FONT_HERSHEY_SIMPLEX, 1, color, 1)
        cv.putText(image, row1, (0,100) , cv.FONT_HERSHEY_SIMPLEX, 1, color, 1)

    def edit(self, input, token):
        if input == ButtonInput.OK:
            return None

        return "A"
