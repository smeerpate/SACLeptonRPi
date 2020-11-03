from ButtonInput import ButtonInput

class Setting(object):
    """description of class"""

    def __init__(self, name):
        self.name = name

    def show(self, image):
        print("Showing " + self.name)

    def edit(self, input, token):
        print("Editing")

