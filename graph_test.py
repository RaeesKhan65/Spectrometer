import sys
from PyQt5 import QtCore, QtWidgets, uic
from PyQt5.QtCore import QRunnable, QThread, QThreadPool, pyqtSignal
from Model.Model_threading import AndorIdus, Stepper
from Views.MainView_threading import MainView

class App(QtWidgets.QApplication):
    def __init__(self, sys_argv):
        super(App, self).__init__(sys_argv)
        self.model = AndorIdus()
        self.stepper = Stepper()
        self.model.SetVerbose(False)
        self.model._width = 1024
        self.model._height = 127
        self.main_view = MainView(self.model, self.stepper) 
        self.main_view.show()
        self.main_view.plot_max_data(sys.argv[1])
        print("showing main view")

if __name__ == '__main__':
    app = App(sys.argv)
    sys.exit(app.exec_())
