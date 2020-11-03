import RPi.GPIO as GPIO
from .ButtonInput import ButtonInput

class InputManager(object):
    """description of class"""

    def __init__(self, up, ok, down):
        self._buttonInput = None
        
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(up, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)  
        GPIO.add_event_detect(up, GPIO.RISING, callback=self._onUpPressed)

        GPIO.setup(ok, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)  
        GPIO.add_event_detect(ok, GPIO.RISING, callback=self._onOkPressed)

        GPIO.setup(down, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)  
        GPIO.add_event_detect(down, GPIO.RISING, callback=self._onDownPressed)

    def _onUpPressed(self, channel):
        print("Up")
        self._buttonInput = ButtonInput.UP

    def _onOkPressed(self, channel):
        print("Ok")
        self._buttonInput = ButtonInput.OK

    def _onDownPressed(self, channel):
        print("Down")
        self._buttonInput = ButtonInput.DOWN

    def hasInput(self):
        return self._buttonInput != None

    def read(self):
        input = self._buttonInput
        self._buttonInput = None
        return input