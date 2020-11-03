from Setting import Setting
from ButtonInput import ButtonInput
import cv2 as cv

class NumberSetting(Setting):
    """description of class"""

    def __init__(self, name):
        super().__init__(name)
        self.value = 1
        self.unit = ""
        self.step = 1
        self.decimals = 1
        self.minimum = 0
        self.maximum = 10

    def show(self, image):
        color = (255, 0, 0)
        cv.putText(image, ("{:." + str(self.decimals) + "f}").format(round(self.value, self.decimals)) + self.unit, (0,50) , cv.FONT_HERSHEY_SIMPLEX, 1, color, 1)

    def edit(self, input, token):
        if input == ButtonInput.OK:
            return None

        if input == ButtonInput.UP:
            newValue = self.value + self.step
            if newValue <= self.maximum:
                self.value = newValue

        if input == ButtonInput.DOWN:
            newValue = self.value - self.step
            if newValue > self.minimum:
                self.value = newValue

        return "A"
