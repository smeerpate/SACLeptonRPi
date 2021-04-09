import RPi.GPIO as GPIO

class LedDriver(object):
    """description of class"""

    def __init__(self, r, g, b):
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(r, GPIO.OUT)
        self.r = GPIO.PWM(r, 100)
        self.r.start(0)

        GPIO.setup(g, GPIO.OUT)
        self.g = GPIO.PWM(g, 100)
        self.g.start(0)

        GPIO.setup(b, GPIO.OUT)
        self.b = GPIO.PWM(b, 100)
        self.b.start(100)

        

    def output(self, red, green, blue, brightness):
        #print("R: " + str(red) + ", G: " + str(green) + ", B: " + str(blue) + " at " + str(brightness) + "%")
        self.r.ChangeDutyCycle(self._getDutyCycle(red))
        self.g.ChangeDutyCycle(self._getDutyCycle(green))
        self.b.ChangeDutyCycle(self._getDutyCycle(blue))

    def stop():
        self.r.stop()
        self.g.stop()
        self.b.stop()
        GPIO.cleanup()

    def _getDutyCycle(self, color):
        return (color - 0) * (100 - 0) / (255 - 0) + 0
