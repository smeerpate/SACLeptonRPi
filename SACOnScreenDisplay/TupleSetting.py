from .Setting import Setting

class TupleSetting(Setting):
    """description of class"""

    def __init__(self, name):
        super().__init__(name)
        self.value = (0, 0)

    def show(self, image):
        return


