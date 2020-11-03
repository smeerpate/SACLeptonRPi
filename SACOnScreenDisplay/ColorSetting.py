from Setting import Setting
from ButtonInput import ButtonInput
import cv2 as cv

class ColorSetting(Setting):
    """description of class"""

    def __init__(self, name):
        super().__init__(name)
        self.red = 0
        self.green = 0
        self.blue = 0

    def show(self, image):
        font = cv.FONT_HERSHEY_SIMPLEX
        fontScale = 1
        fontThickness = 1
        x = 0        
        
        cv.putText(image, str(self.red), (x, 50), font, fontScale, (0, 0, 255), fontThickness)

        (width, height) = cv.getTextSize(str(self.red) + " ", font, fontScale, fontThickness)
        x = x + width[0]

        cv.putText(image, str(self.green), (x, 50), font, fontScale, (0, 255, 0), fontThickness)

        (width, height) = cv.getTextSize(str(self.green) + " ", font, fontScale, fontThickness)
        x = x + width[0]

        cv.putText(image, str(self.blue), (x, 50), font, fontScale, (255, 0, 0), fontThickness)

    def edit(self, input, token):
        if token == None or token == "R":
            if input == ButtonInput.OK:
                return "G"
            else:
                self.red = self.__editColorComponent(self.red, input)
                return "R"

        if token == "G":
            if input == ButtonInput.OK:
                return "B"
            else:
                self.green = self.__editColorComponent(self.green, input)
                return "G"

        if token == "B":
            if input == ButtonInput.OK:
                return None
            else:
                self.blue = self.__editColorComponent(self.blue, input)
                return "B"

    def __editColorComponent(self, component, input):

        if input == ButtonInput.UP:
            if component < 255:
                component = component + 1

        if input == ButtonInput.DOWN:
            if component > 0:
                component = component - 1

        return component