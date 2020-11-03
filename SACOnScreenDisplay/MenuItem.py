import cv2 as cv

class MenuItem(object):
    """description of class"""

    def __init__(self):
        self.name = None
        self.menuItems = None
        self.setting = None

    def getDisplayName(self):
        if self.setting != None:
            return self.setting.name
        else:
            return self.name
