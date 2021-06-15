from PyQt5 import QtWidgets
from matplotlib.backends.backend_qt4agg \
import FigureCanvasQTAgg as FigureCanvas
import matplotlib
import matplotlib.ticker as plticker
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt4agg import (
    NavigationToolbar2QT as NavigationToolbar
)

class MplCanvas(FigureCanvas):
    """Class to represent the FigureCanvas widget"""
    def __init__(self):
        # setup Matplotlib Figure and Axis
        matplotlib.rcParams.update({'font.size': 8})
        self.fig = Figure()
        self.fig.set_tight_layout(True)
        #loc = plticker.MultipleLocator(base=50)
        #self.ax.xaxis.set_major_locator(loc)
        # initialization of the canvas
        FigureCanvas.__init__(self, self.fig)
        # we define the widget as expandable
        FigureCanvas.setSizePolicy(self,
                                   QtWidgets.QSizePolicy.Expanding,
                                   QtWidgets.QSizePolicy.Expanding)
        FigureCanvas.updateGeometry(self)

class MplWidget(QtWidgets.QWidget):
    """Widget defined in Qt Designer"""
    def __init__(self, parent = None):
        QtWidgets.QWidget.__init__(self, parent)
        self.canvas = MplCanvas()
        self.vbl = QtWidgets.QVBoxLayout()
        self.vbl.addWidget(self.canvas)
        #self.vbl.addWidget(self.toolbar)
        self.setLayout(self.vbl)
