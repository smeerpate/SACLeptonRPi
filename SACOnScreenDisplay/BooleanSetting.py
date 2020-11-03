from Setting import Setting
from ButtonInput import ButtonInput
import cv2 as cv

class BooleanSetting(Setting):
    """description of class"""

    def __init__(self, name):
        super().__init__(name)
        self.value = False

    def show(self, image):
        font = cv.FONT_HERSHEY_SIMPLEX
        fontScale = 1
        fontThickness = 1
        x = 0        
        
        if self.value == True:
            cv.putText(image, "True", (x, 50), font, fontScale, (0, 255, 0), fontThickness)
        else:
            cv.putText(image, "False", (x, 50), font, fontScale, (0, 0, 255), fontThickness)

    def edit(self, input: ButtonInput, token: str) -> str:
        if input == ButtonInput.OK:
            return None
        if input == ButtonInput.UP or input == ButtonInput.DOWN:
            self.value = not self.value

        return "A"

