import sys
from PyQt5 import QtCore, QtWidgets, uic
from PyQt5.QtCore import QRunnable, QThread, QThreadPool, pyqtSignal
from Model.Model_threading import AndorIdus, Stepper
from Views.MainView_threading import MainView

class App(QtWidgets.QApplication):
    def __init__(self, sys_argv):
        super(App, self).__init__(sys_argv)
        self.model = AndorIdus()
        self.model.SetVerbose(False)
        self.stepper = Stepper()
        self.main_view = MainView(self.model, self.stepper)
        self.main_view.show()
        print("showing main view")

if __name__ == '__main__':
    app = App(sys.argv)
    sys.exit(app.exec_())
