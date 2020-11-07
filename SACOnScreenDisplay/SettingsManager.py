import os
import jsonpickle

from .Settings import Settings
from .Setting import Setting
from .NumberSetting import NumberSetting
from .BooleanSetting import BooleanSetting
from .TupleSetting import TupleSetting
from .ColorSetting import ColorSetting

class SettingsManager(object):
    """description of class"""

    def __init__(self):
        self.settingsFile = "/home/pi/SACLeptonRPi/OSD Settings.json"
        self.settings = None

    def getSettings(self):
        if self.settings != None:
            return self.settings
        else:
            if os.path.isfile(self.settingsFile):
                print("Reading settings from file: " + self.settingsFile)
                f = open(self.settingsFile, "r")
                self.settings = jsonpickle.decode(f.read())
                f.close()
            else:
                print(self.settingsFile + " is not a file, getting default settings")
                self.settings = self.__getDefaultSettings()
                self.saveSettings(self.settings)

        #self.__printSettings()
        return self.settings

    def __printSettings(self):
        print("Treshold: " + str(self.settings.threshold.value) + str(self.settings.threshold.unit))
        print("Offset: " + str(self.settings.offset.value))
        print("MeasurementsPerMean: " + str(self.settings.measurementsPerMean.value))

    def __getDefaultSettings(self):
        settings = Settings()
        showMeanTemperature = BooleanSetting("Show mean temperature")
        showMeanTemperature.value = False
        settings.showMeanTemperature = showMeanTemperature

        showFoundFace = BooleanSetting("Show found face")
        showFoundFace.value = True
        settings.showFoundFace = showFoundFace

        showWarmestZones = BooleanSetting("Show warmest zones")
        showWarmestZones.value = False
        settings.showWarmestZones = showWarmestZones

        screenPosition = TupleSetting("Screen position")
        screenPosition.value = (0, 50)
        settings.screenPosition = screenPosition

        screenDimensions = TupleSetting("Screen dimensions")
        screenDimensions.value = (360, 270)
        settings.screenDimensions = screenDimensions

        threshold = NumberSetting("Threshold")
        threshold.value = 35.7
        threshold.unit = 'deg'
        threshold.step = 0.1
        threshold.decimals = 1
        threshold.minimum = 35
        threshold.maximum = 40
        settings.threshold = threshold

        offset = NumberSetting("Offset")
        offset.value = 0.5
        offset.unit = 'deg'
        offset.step = 0.1
        offset.decimals = 1
        offset.minimum = 0
        offset.maximum = 2
        settings.offset = offset

        epsilon = NumberSetting("Epsilon")
        epsilon.value = 1.5
        epsilon.unit = "err"
        epsilon.step = 0.1
        epsilon.decimals = 1
        epsilon.minimum = 0
        epsilon.maximum = 3
        settings.epsilon = epsilon

        measurementsPerMean = NumberSetting("Measurements per mean")
        measurementsPerMean.value = 3
        measurementsPerMean.unit = "m/m"
        measurementsPerMean.step = 1
        measurementsPerMean.decimals = 0
        measurementsPerMean.minimum = 1
        measurementsPerMean.maximum = 5
        settings.measurementsPerMean = measurementsPerMean

        brightness = NumberSetting("Brightness")
        brightness.value = 75
        brightness.unit = "%"
        brightness.step = 1
        brightness.decimals = 0
        brightness.minimum = 0
        brightness.maximum = 100
        settings.brightness = brightness

        alarmColor = ColorSetting("Alarm color")
        alarmColor.red = 255
        alarmColor.green = 0
        alarmColor.blue = 0
        settings.alarmColor = alarmColor

        okColor = ColorSetting("OK color")
        okColor.red = 0
        okColor.green = 255
        okColor.blue = 0
        settings.okColor = okColor

        idleColor = ColorSetting("Idle color")
        idleColor.red = 255
        idleColor.green = 255
        idleColor.blue = 255
        settings.idleColor = idleColor

        return settings

    def saveSettings(self, settings):
        f = open(self.settingsFile, "w")
        f.write(jsonpickle.encode(settings))
        f.close()