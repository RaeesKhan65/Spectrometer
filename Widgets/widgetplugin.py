from PyQt5 import QtWidgets, QtDesigner

class CustomTextBoxPlugin(QtDesigner.QPyDesignerCustomWidgetPlugin):
    def __init__(self, parent=None):
        super(CustomTextBoxPlugin).__init__(parent)
        self.initialized = False

    def initialize(self, core):
        if self.initialized:
            return

        self.initialized = True
