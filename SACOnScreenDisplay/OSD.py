import cv2 as cv

from .MenuItem import MenuItem
from .Settings import Settings
from .InputManager import InputManager
from .SettingsManager import SettingsManager
from .ButtonInput import ButtonInput
from .DisplayMixer import DisplayMixer

class OSD(object):
    """description of class"""

    def __init__(self, inputManager, settingsManager, displayMixer):
        self.inputManager = inputManager
        self.settingsManager = settingsManager
        self.displayMixer = displayMixer
        self.settings = None
        
        self.areMenusActive = False
        self.activeMenu = None
        self.selectedMenu = None
        self.root = None
        self.editToken = None

    def __handleMenuNavigation(self):
        keyPressed = self.inputManager.read()
        activeMenuIndex = self.selectedMenu.menuItems.index(self.activeMenu)    
        
        if keyPressed == ButtonInput.UP:
            if activeMenuIndex > 0:
                self.activeMenu = self.selectedMenu.menuItems[activeMenuIndex - 1]

        if keyPressed == ButtonInput.DOWN:
            if activeMenuIndex < len(self.selectedMenu.menuItems) - 1:
                self.activeMenu = self.selectedMenu.menuItems[activeMenuIndex + 1]

        if keyPressed == ButtonInput.OK:
            self.selectedMenu = self.activeMenu
            if self.selectedMenu.menuItems != None and len(self.selectedMenu.menuItems) > 0 :
                self.activeMenu = self.selectedMenu.menuItems[0]

    def __initMenus(self):
        if self.root == None:
            self.root = MenuItem()
            self.root.menuItems = self.__createMenus()

        self.activeMenu = self.root.menuItems[0]
        self.selectedMenu = self.root

    def __createMenus(self):
        self.settings = self.settingsManager.getSettings()

        smtMenu = MenuItem()
        smtMenu.setting = self.settings.showMeanTemperature

        sffMenu = MenuItem()
        sffMenu.setting = self.settings.showFoundFace

        swzMenu = MenuItem()
        swzMenu.setting = self.settings.showWarmestZones

        spMenu = MenuItem()
        spMenu.setting = self.settings.screenPosition

        sdMenu = MenuItem()
        sdMenu.setting = self.settings.screenDimensions

        viewItems = MenuItem()
        viewItems.name = "View"
        viewItems.menuItems = [smtMenu, sffMenu, swzMenu, spMenu, sdMenu]

        thresholdMenu = MenuItem()
        thresholdMenu.setting = self.settings.threshold

        offsetMenu = MenuItem()
        offsetMenu.setting = self.settings.offset

        epsilonMenu = MenuItem()
        epsilonMenu.setting = self.settings.epsilon

        mpmMenu = MenuItem()
        mpmMenu.setting = self.settings.measurementsPerMean

        measureItems = MenuItem()
        measureItems.name = "Measure"
        measureItems.menuItems = [thresholdMenu, offsetMenu, epsilonMenu, mpmMenu]

        brightnessMenu = MenuItem()
        brightnessMenu.setting = self.settings.brightness

        alarmColorMenu = MenuItem()
        alarmColorMenu.setting = self.settings.alarmColor

        okColorMenu = MenuItem()
        okColorMenu.setting = self.settings.okColor

        idleColorMenu = MenuItem()
        idleColorMenu.setting = self.settings.idleColor

        ledsItems = MenuItem()
        ledsItems.name = "LEDs"
        ledsItems.menuItems = [brightnessMenu, alarmColorMenu, okColorMenu, idleColorMenu]

        return [viewItems, measureItems, ledsItems]

    def __exitMenus(self):
        self.areMenusActive = False

    def __editSetting(self, setting, image):
        setting.show(image)
        #read GPIO
        buttonInput = self.inputManager.read()
        #edit + current stage
        self.editToken = setting.edit(buttonInput, self.editToken)

        if self.editToken == None:
            return True

        return False

    def run(self, image):
        print("Running OSD...")
        if not self.areMenusActive:
            keyPressed = self.inputManager.read()
            if keyPressed != None:
                self.areMenusActive = True
                self.__initMenus()
        else:
            # Display the resulting frame
            if self.selectedMenu.menuItems != None and len(self.selectedMenu.menuItems) > 0:
                menuOffsetY = 30
                menuItemNumber = 0
                for menuItem in self.selectedMenu.menuItems:
                    color = (0, 0, 255, 255)
                    if menuItem == self.activeMenu:
                        color = (0, 255, 0, 255)
                    cv.putText(image, menuItem.getDisplayName(), (20,20 + menuOffsetY * menuItemNumber), cv.FONT_HERSHEY_SIMPLEX, 1, color, 1)
                    menuItemNumber += 1
                self.__handleMenuNavigation()
            else:        
                if self.__editSetting(self.selectedMenu.setting, image):
                    self.settingsManager.saveSettings(self.settings)
                    self.__exitMenus()

    def isRunning(self):
        return self.areMenusActive