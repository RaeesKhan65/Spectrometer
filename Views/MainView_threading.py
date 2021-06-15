# to do: write function to handle implementation of generic user input
# instead of having @properties for every variable. can do this using
# __getattr__ and __setattr__
#
# to add functionality with Arduino, you need to run the file
# StepperMotorArduinoCode.ino

import os.path
import time
import datetime
import inspect
import csv
from PyQt5.QtCore import pyqtSlot, pyqtSignal
from PyQt5 import QtCore, QtWidgets, uic, QtGui
from pprint import pprint
from scipy.signal import argrelextrema
from Model import threads, calibrate
from PIL import Image
import numpy as np
from matplotlib.backends.backend_qt4agg import (
    FigureCanvasQTAgg as figureCanvas
)
from matplotlib.backends.backend_qt4agg import (
    NavigationToolbar2QT as NavigationToolbar
)
from matplotlib.figure import Figure
import matplotlib.gridspec as gridspec
from matplotlib.widgets import SpanSelector

qtCreatorFile = \
    r"C:\Users\duttlab6b\PycharmProjects\Spectrometer\modified_spectrometer.ui"
Ui_MainWindow, QtBaseClass = uic.loadUiType(qtCreatorFile)


class MainView(QtWidgets.QMainWindow, Ui_MainWindow):
    """
    The view/controller for the system. When the app runs,
    it creates an instance of Model() and passes it to MainView.
    (i.e. self.model is a Model() instance)
    """

    def __init__(self, model, stepper):
        self.model = model  # model is the camera
        self.stepper = stepper
        self.model._view_funcs.append(self.update_ui_from_model)

        self.get_data_button_has_been_pushed = False
        self.want_to_shut_down = False
        self.my_imshow = None  # this holds the bmp imshow
        self.pix_to_wavelength = lambda x: x
        self.pix_to_wavelength_display = lambda x: x
        # ^this is a temporary pixel-to-wavelength function.
        self.m = None
        self.b = None
        # ^slope and intercept of pixel-to-wavelength function.
        self.y_min = 10
        self.y_max = None
        self.y_min_display = None
        self.y_max_display = None
        # ^ymin and ymax defines the range of vertical pixels we use
        self.display_data = None
        # ^holds x and y data for the plot on the display tab
        self.display_im = None
        # ^holds the entire 127x1024 display image

        self.should_i_plot = False
        super(MainView, self).__init__()
        self.setupUi(self)
        self.data_path = "H:\Entanglement Experiments\Data\Spectrum"
        self.calibration_spectrum_file = \
            self.calibration_spectrum_path.text()
        self.background_file = None
        # ^bg noise file
        self.display_file = None

        self.cal_file = None  # calibration file
        self.cal_file_display = None  # calibration file for display

        # set calibration pixel box to integer-only
        self.onlyInt = QtGui.QIntValidator()
        self.onlyInt.setBottom(0)
        self.pixel_input_box.setValidator(self.onlyInt)
        self.y_min_box.setValidator(self.onlyInt)
        self.y_max_box.setValidator(self.onlyInt)

        # set up the plots
        gs = gridspec.GridSpec(2, 1, height_ratios=[6, 1])
        # ^want the image to be half as tall as the graph
        #  for sleekness

        self.graph_box.canvas.ax1 = \
            self.graph_box.canvas.fig.add_subplot(gs[0])
        self.graph_box.canvas.ax2 = \
            self.graph_box.canvas.fig.add_subplot(
                gs[1], sharex=self.graph_box.canvas.ax1)

        self.graph_box.canvas.ax2.tick_params(
            axis='both',
            bottom=False,
            left=False,
            labelbottom=False,
            labelleft=False)
        self.graph_box.canvas.ax2.set_aspect(1)
        self.graph_box.toolbar = NavigationToolbar(
            self.graph_box.canvas, self)
        self.graph_box.vbl.addWidget(self.graph_box.toolbar)
        self.graph_box.span = SpanSelector(
            self.graph_box.canvas.ax2, self.onselect, 'vertical')
        # self.graph_box.vbl.addWidget(self.graph_box.span)

        self.display_box.canvas.ax1 = \
            self.display_box.canvas.fig.add_subplot(gs[0])
        self.display_box.canvas.ax2 = \
            self.display_box.canvas.fig.add_subplot(
                gs[1], sharex=self.display_box.canvas.ax1)
        self.display_box.span = SpanSelector(
            self.display_box.canvas.ax2, self.onselect_display,
            'vertical')

        self.display_box.canvas.ax2.tick_params(
            axis='both',
            bottom=False,
            left=False,
            labelbottom=False,
            labelleft=False)
        self.display_box.canvas.ax2.set_aspect(1)
        self.display_box.toolbar = NavigationToolbar(
            self.display_box.canvas, self)
        self.display_box.vbl.addWidget(self.display_box.toolbar)

        self.calibration_graph.canvas.ax1 = \
            self.calibration_graph.canvas.fig.add_subplot(111)
        self.calibration_graph.canvas.ax1.tick_params(
            left=False, labelleft=False)
        self.calibration_graph.toolbar = NavigationToolbar(
            self.calibration_graph.canvas, self)
        self.calibration_graph.vbl.addWidget(
            self.calibration_graph.toolbar)

        self.calibration_data_box.canvas.ax1 = \
            self.calibration_data_box.canvas.fig.add_subplot(121)
        self.calibration_data_box.canvas.ax2 = \
            self.calibration_data_box.canvas.fig.add_subplot(122)
        self.calibration_data_box.canvas.ax2.tick_params(
            left=False, labelleft=False)

        self.calibration_data_box.toolbar = NavigationToolbar(
            self.calibration_data_box.canvas, self)
        self.calibration_data_box.vbl.addWidget(
            self.calibration_data_box.toolbar)

        # set labels
        # self.graph_box.canvas.ax.set_xlim((0,1024))
        # self.graph_box.canvas.ax1.set_xlabel('Wavelength (nm)')
        self.graph_box.canvas.ax1.set_xlabel('Pixel')
        self.graph_box.canvas.ax1.set_ylabel('Count (mean)')

        self.calibration_graph.canvas.ax1.set_xlabel('Pixel')

        self.calibration_data_box.canvas.ax1.set_xlabel(
            'Peak location (pixel)')
        self.calibration_data_box.canvas.ax1.set_ylabel(
            'Peak location (nm)')
        self.calibration_data_box.canvas.ax2.set_xlabel(
            'Pixel')

        # initialize stderr output thread, which runs the whole time
        # and prints the response from the camera. also updates the
        # temperature display box.
        self.stderrThread = threads.StderrThread(self.model)
        self.stderrThread.s_init()
        self.stderrThread.start()

        # set up other threads without starting them
        self.workerThread = None
        self.warmupThread = None

        # some properties of input boxes
        self.exposure_time_box.undoAvailable = True
        self.step_size_box.undoAvailable = True
        self.temperature_box.undoAvailable = True
        self.cal_path.undoAvailable = True
        self.display_path.undoAvailable = True
        self.temperature_display.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.display_temperature = '---'  # temperature shown on LCD display
        self.exposure_led.setValue(False)

        # make connections
        self.trigger_selector.currentIndexChanged.connect(
            self.change_trigger_type)
        self.temperature_box.editingFinished.connect(
            self.change_temperature)
        self.step_size_box.editingFinished.connect(
            self.change_step_size)
        self.exposure_time_box.editingFinished.connect(
            self.change_exposure_time)
        self.data_name_box.editingFinished.connect(
            self.change_data_name)
        self.y_min_box.editingFinished.connect(
            self.change_ymin)
        self.y_max_box.editingFinished.connect(
            self.change_ymax)
        self.select_path.clicked.connect(self.selectFile)
        self.select_background_button.clicked.connect(
            self.select_background_file)
        self.add_display_data_button.clicked.connect(
            lambda: self.select_display_file(add=True))
        self.select_display_data_button.clicked.connect(
            lambda: self.select_display_file(add=False))
        self.select_cal_file_button.clicked.connect(
            self.select_cal_file_display)
        self.load_cal_button.clicked.connect(
            self.select_cal_file)
        self.save_cal_button.clicked.connect(
            self.save_cal_file)
        self.save_data_button.clicked.connect(
            self.save_data)
        self.reset_display_button.clicked.connect(
            self.reset_display)
        self.reset_display_cal_button.clicked.connect(
            self.reset_display_cal)
        self.maximize_button.clicked.connect(self.maximize)
        self.start_button.clicked.connect(self.collect_data)
        self.shut_down_button.clicked.connect(self.shut_down)
        self.continue_taking_data_box.toggled.connect(
            self.change_checked_type)
        self.stop_cooler_button.clicked.connect(
            self.stop_worker_thread)
        self.move_motor_up.clicked.connect(
            lambda: self.move_motor(self.step_size, 'U'))
        self.move_motor_down.clicked.connect(
            lambda: self.move_motor(self.step_size, 'D'))
        self.load_calibration_spectrum_button.clicked.connect(
            self.select_calibration_spectrum_file)
        self.spectrum_reset.clicked.connect(
            self.spectrum_reset_function)
        self.calibration_reset.clicked.connect(
            self.calibration_reset_function)
        self.calibration_button.clicked.connect(
            self.implement_calibration)
        self.reset_background_button.clicked.connect(
            self.reset_background)
        # ^eventually want to make 'reset' button reset the
        # calibration.

    def change_ymin(self):
        temp = int(str(self.y_min_box.text()))
        if temp >= 0 and temp < 127:
            self.y_min_display = temp
            self.update_plot(which_plot='display')

    def change_ymax(self):
        temp = int(str(self.y_max_box.text()))
        if temp >= 0 and temp < 127:
            self.y_max_display = temp
            self.update_plot(which_plot='display')

    def maximize(self):
        """
        Takes the 1024x127 image array and finds the y range that
        maximizes the contrast in data (minimizes blank rows)

        This function was originally in calibrate.py
        """
        if self.display_im is not None:
            img_array = self.display_im
        else:
            raise Exception("Nothing to maximize.")

        previous_max = 0
        y_lower = 0
        y_upper = 1

        for l in range(127):
            for m in range(l + 1, 127 - l):
                y1, y2 = (l, m)
                print("#### l {} m {} ####".format(l, m))
                self.y_min_display = y1
                self.y_max_display = y2

                y_vals = [
                    np.mean(img_array[y1:y2, k]) for k in range(1024)]

                # max_indices = list(
                #    argrelextrema(
                #        np.array(y_vals), np.greater, order=3))[0]
                ## ^order is 3 so that single bright pixels don't count
                # maxima = [y_vals[i] for i in max_indices]
                # highest_max = sorted(maxima)[-1]
                highest_max = sorted(y_vals)[-1]

                if highest_max > previous_max:
                    previous_max = highest_max
                    y_lower = l
                    y_upper = m

        self.y_min_display = y_lower
        self.y_max_display = y_upper

        self.update_plot(which_plot='display')

    def save_data(self):
        """
        Saves the data on the display page.
        """
        data_to_save = list(zip(*self.display_data))

        name = QtWidgets.QFileDialog.getSaveFileName(self, 'Save File')
        with open(name + '.txt', 'w') as f:
            writer = csv.writer(f, delimiter='\t')
            writer.writerows(data_to_save)

        print("display file saved.")

    def reset_display(self):
        print("resetting")
        self.y_min_display = None
        self.y_max_display = None

        self.display_path.setText('')
        self.display_box.canvas.ax1.cla()
        self.display_box.canvas.ax2.cla()
        self.display_box.canvas.draw()

    def reset_display_cal(self):
        self.pix_to_wavelength_display = lambda x: x
        self.update_plot(which_plot='display')
        self.cal_path.setText('')

    def onselect(self, y_min, y_max):
        """
        For span selector.
        """
        print("...")
        print("Main graph y range:", y_min, "--->", y_max)
        print("...")
        self.y_min = int(y_min)
        self.y_max = int(y_max)

        self.update_plot(which_plot='main')
        self.graph_box.canvas.ax2.axhline(
            y_min, color='red', linewidth=0.4)
        self.graph_box.canvas.ax2.axhline(
            y_max, color='red', linewidth=0.4)

    def onselect_display(self, y_min, y_max):
        """
        For span selector.
        """
        print("...")
        print("Display y range:", y_min, "--->", y_max)
        print("...")

        self.y_min_display = int(y_min)
        self.y_max_display = int(y_max)

        self.y_min_box.setText(str(self.y_min_display))
        self.y_max_box.setText(str(self.y_max_display))

        self.update_plot(which_plot='display', add=False)
        self.display_box.canvas.ax2.axhline(
            y_min, color='red', linewidth=0.4)
        self.display_box.canvas.ax2.axhline(
            y_max, color='red', linewidth=0.4)

    def reset_background(self):
        self.background_file = None
        self.background_path.setText('')

    def spectrum_reset_function(self):
        """
        Resets the neon spectrum file chosen by the user.
        """
        self.calibration_spectrum_path.setText('')
        self.calibration_spectrum_file = None
        # ^could use getter and setter for this
        self.calibration_reset_function()

    def calibration_reset_function(self):
        print("CALIBRATION RESET FUNCTION")
        self.pixel_input_box.setText('0')

        self.calibration_data_box.canvas.ax1.cla()
        self.calibration_data_box.canvas.ax2.cla()
        # ^doesn't seem to work

        self.pix_to_wavelength = lambda x: x  # resets function

        if os.path.isfile(self.current_plot_file):
            self.graph_box.canvas.ax1.set_xlabel('Pixel')
            self.update_plot(which_plot='main')

        self.calibration_data_box.canvas.draw()
        self.graph_box.canvas.draw()

    def implement_calibration(self, data_list, path_to_file=''):
        """
        Takes the location of the 703.24 nm peak, specified by the
        user on the calibration page, and performs a fit to determine
        the wavelength as a function of pixel.

        Also saves a .cal file that you can load in the future
        without going through the whole calibration process.
        """
        my_peak = int(self.pixel_input_box.text())  # which pixel is this peak at?
        print("peak is", my_peak)

        path_to_neon_calibration_spectrum = \
            r'model\neon_calibration.txt'

        pix_to_wavelength, data_list = calibrate.calibrate(
            self.calibration_spectrum_file,
            path_to_neon_calibration_spectrum,
            which_peak=float(self.peak_location),
            location_of_peak=my_peak)
        # ^pix_to_wavelength is a function. data_list is a bunch
        #  of data that is used in the calibration plots.

        self.pix_to_wavelength = pix_to_wavelength
        print("pix_to_wavelength function updated.")

        [maxima_locations, peak_locations, final_peaks,
         final_x, y_vals, R, self.m, self.b] = data_list

        a1 = self.calibration_data_box.canvas.ax1
        a2 = self.calibration_data_box.canvas.ax2

        a1.cla()
        a2.cla()

        a1.plot(maxima_locations, peak_locations, 'r.')
        a1.plot(maxima_locations, final_peaks)

        a1.text(maxima_locations[-1] / 2, peak_locations[0],
                "R=%.3f" % R, fontsize=6)

        a2.plot(final_x, y_vals, 'k-', label='User spectrum')

        a2.axvline(
            x=peak_locations[0], linewidth=0.5,
            color='green', label='Known peak location')

        for point in peak_locations[1:]:
            a2.axvline(x=point, linewidth=0.5,
                       color='green')

        a2.legend()
        self.calibration_data_box.canvas.draw()
        self.graph_box.canvas.draw()
        xtick_locations = list(range(0, self.model._width,
                                     int(self.model._width / 10)))

        if os.path.isfile(self.current_plot_file):
            # ^e.g. if data has already been taken...
            self.graph_box.canvas.ax1.set_xlabel('Wavelength (nm)')
            self.update_plot(which_plot='main')

        # self.graph_box.canvas.ax2.clear()
        # self.graph_box.canvas.ax2.set_xticks(xtick_locations)
        # self.graph_box.canvas.ax2.set_xticklabels(
        #    [round(i, 2) for i in final_x[::100]])
        # self.graph_box.canvas.ax1.xaxis.set_tick_params(rotation=45)

    def save_cal_file(self):
        """
        Saves a .cal calibration file that can be loaded to get a
        pixel-to-wavelength conversion.
        """
        c_filename = \
            str(self.calibration_spectrum_file).split('.')[:-1]
        c_filename = ''.join(c_filename) + '.cal'

        if self.m and self.b:  # i.e. if the calibration has been done
            with open(c_filename, 'w') as calfile:
                calfile.write('{} {}'.format(self.m, self.b))
            print("calibration file saved.")
        else:
            print("calibration doesn't exist")

    def move_motor(self, step_size, direction):
        """
        Starts a thread to move the motor, then immediately kills the
        thread (rather, the thread kills itself).
        """
        self.stepperThread = threads.MoveMotorThread(
            self.stepper, step_size, direction)
     #   self.connect(
      #      self.stepperThread, QtCore.SIGNAL("done_stepping"),
       #     self.end_stepper_thread)
        self.stepperThread.done_stepping.connect(self.end_stepper_thread)

       # self.connect(
       #     self.stepperThread, QtCore.SIGNAL("stepping"),
       #     self.freeze_stepper_buttons)
        self.stepperThread.stepping.connect(self.freeze_stepper_buttons)
        self.stepperThread.start()
        print("step made.")

    # the following are just a bunch of setters and getters
    @property  # getter
    def trigger_type(self):  # 0 = internal; 1 = external
        t_type = self.trigger_selector.currentIndex()
        return t_type

    @trigger_type.setter  # setter
    def trigger_type(self, value):
        if value in [0, 1]:
            self.update_model()
            return value
        else:
            pass

    @property
    def peak_location(self):
        p_loc = self.peak_input_box.text()
        return p_loc

    @property
    def temperature(self):
        temp = int(self.temperature_box.text())
        return temp

    @temperature.setter
    def temperature(self, value):
        try:
            temp = int(float(value))
            self.temperature_box.clear()
            self.temperature_box.insert(str(temp))
            return temp
        except Exception:
            print("temperature must be a number")
            self.temperature_box.undo()

    @property  # step size for stepper motor
    def step_size(self):
        temp = str(self.step_size_box.text())
        return temp

    @step_size.setter  # must be integer
    def step_size(self, value):
        try:
            temp = int(float(value))
            temp = str(temp)
            if len(temp) != 5:
                if len(temp) > 5:
                    print("step_size must be 5 digits")
                    raise AttributeError
                else:
                    temp = '0' * (5 - len(temp)) + temp

            self.step_size_box.clear()
            self.step_size_box.insert(temp)
            return temp
        except Exception:
            print("step_size must be a number")
            self.step_size_box.undo()

    @property
    def exposure_time(self):
        temp = float(self.exposure_time_box.text())
        return temp

    @exposure_time.setter
    def exposure_time(self, value):
        try:
            temp = float(value)
            assert temp >= 0
            self.exposure_time_box.clear()
            self.exposure_time_box.insert(str(temp))
            return temp
        except Exception:
            print("exposure time must be a number")
            self.exposure_time_box.undo()

    @property
    def data_name(self):
        return self.data_name_box.text()

    @data_name.setter
    def data_name(self, value):
        self.update_model()
        return value

    @property
    def current_display_file(self):
        if os.path.isfile(self.display_path.text()):
            return self.display_path.text()
        else:
            print("currently no valid display file.")

    @property
    def current_plot_file(self):
        if os.path.isfile(
                self.data_path + '\\' + self.data_name + '.txt'):
            return self.data_path + '\\' + self.data_name + '.txt'
        else:
            print("not a valid plot file.")

    @property
    def cooler_on(self):
        return self.model._cooler_on

    @cooler_on.setter
    def cooler_on(self, value):
        if value == True:
            self.cooler_status.setText('Cooler is on.')
            self.stop_cooler_button.setEnabled(True)
        elif value == False:
            self.cooler_status.setText('Cooler is off.')
            self.stop_cooler_button.setEnabled(False)
        else:
            pass
        return value

    @property
    def currently_taking_data(self):
        return self.model._currently_taking_data

    @currently_taking_data.setter
    def currently_taking_data(self, value):
        self.status_led.setState(value)
        return value

    @property
    def display_temperature(self):
        return self.temperature_display.value()

    @display_temperature.setter
    def display_temperature(self, value):  # value should be an int
        if value:  # i.e. if temperature is not None
            self.temperature_display.display(value)
            return value
        else:
            pass

    # determines whether the cooler shuts off after a measurement
    @property
    def continue_taking_data(self):
        return self.continue_taking_data_box.isChecked()

    @continue_taking_data.setter  # do we need a setter?
    def continue_taking_data(self, value):  # value is a bool
        self.continue_taking_data_box.setChecked(value)
        return value

    def start(self):
        """
        Right now, this function is called when the 'get data' button
        is pushed.
        """
        # make sure the threads aren't already running:
        self.model.emergency_stop_cooling = False
        self.stop()
        if self.workerThread:
            print("worker thread:", self.workerThread.isRunning())
        if self.stderrThread:
            print("stderr thread:", self.stderrThread.isRunning())
        if self.warmupThread:
            print("warmup thread:", self.warmupThread.isRunning())

        # initialize threads
        self.workerThread = threads.WorkerThread(self.model)
        self.warmupThread = threads.WarmupThread(self.model)

        # connect signals
        #  self.connect(
        #      self.workerThread, QtCore.SIGNAL('Dummy'),
        #      self.acknowledge_signal)  # camera = model
        self.workerThread.dummy.connect(self.acknowledge_signal)
        #   self.connect(
        #      self.workerThread, QtCore.SIGNAL('Stop'),
        #     self.stop)
        self.workerThread.Stop.connect(self.stop)
        #  self.connect(
        #      self.workerThread, QtCore.SIGNAL(
        #          'toggle_parameter_lock(bool)'),
        #      self.toggle_parameter_lock)
        self.workerThread.toggle_parameter_lock.connect(self.toggle_parameter_lock)
        #  self.connect(
        #      self.warmupThread, QtCore.SIGNAL(
        #          'toggle_parameter_lock(bool)'),
        #      self.toggle_parameter_lock)
        self.warmupThread.toggle_parameter_lock.connect(self.toggle_parameter_lock)
        #  self.connect(
        #      self.workerThread, QtCore.SIGNAL('data_acquistion_done'),
        #      self.data_acqusition_done)
        self.workerThread.data_acquistion_done.connect(self.data_acqusition_done)
        #  self.connect(
        #      self.workerThread, QtCore.SIGNAL(
        #          'save_as_bmp(PyQt_PyObject, int, int, PyQt_PyObject)'),
        #      self.save_as_bmp)
        self.workerThread.save_as_bmp.connect(self.save_as_bmp)
        #  self.connect(
        #      self.workerThread, QtCore.SIGNAL(
        #          'save_as_txt(PyQt_PyObject, PyQt_PyObject)'),
        #      self.save_as_txt)
        self.workerThread.save_as_txt.connect(self.save_as_txt)
        #  self.connect(
        #      self.workerThread, QtCore.SIGNAL('make_plot(PyQt_PyObject)'),
        #      self.plot_max_data)
        self.workerThread.make_plot.connect(self.plot_max_data)
        #  self.connect(
        #      self.stderrThread, QtCore.SIGNAL('update_stderr'),
        #      self.update_stderr)
        self.stderrThread.update_stderr.connect(self.update_stderr)
        #  self.connect(
        #      self.stderrThread, QtCore.SIGNAL('update_temp(int)'),
        #      self.update_temp)
        self.stderrThread.update_temp.connect(self.update_temp)
        # self.connect(
        #     self.warmupThread, QtCore.SIGNAL("warming_done"),
        #     self.shut_down_if_necessary)
        self.warmupThread.warming_done.connect(self.shut_down_if_necessary)

        self.workerThread.s_init()
        self.workerThread.start()

    def stop(self):
        """
        Kills threads if they're not already dead.
        """
        time.sleep(0.1)
        print("stop called.")
        curframe = inspect.currentframe()
        calframe = inspect.getouterframes(curframe, 2)
        print('stop caller name:', calframe[1][3])
        if self.workerThread is not None:
            print("killing worker thread.")
            self.workerThread.end_this_thread()
            self.workerThread.wait()
        if self.warmupThread is not None:
            print("killing warmup thread.")
            self.warmupThread.end_this_thread()
            self.warmupThread.wait()

    def hard_stop(self):
        """
        Kills threads if they're not already dead, and sets them to None.
        """
        time.sleep(0.1)
        print("stop called.")
        if self.workerThread is not None:
            print("hard killing worker thread.")
            self.workerThread.end_this_thread()
            time.sleep(0.1)
            self.workerThread.wait()
            self.workerThread = None
        if self.stderrThread is not None:
            print("hard killing stderr thread.")
            self.stderrThread.end_this_thread()
            time.sleep(0.1)
            self.stderrThread.wait()
            self.stderrThread = None
        if self.warmupThread is not None:
            print("hard killing warmup thread.")
            self.warmupThread.end_this_thread()
            time.sleep(0.1)
            self.warmupThread.wait()
            self.warmupThread = None
        # if self.temperatureThread is not None:
        #    print "hard killing temperature thread."
        #    self.temperatureThread.end_this_thread()
        #    time.sleep(0.1)
        #    #self.temperatureThread.wait()
        #    self.temperatureThread = None

    def stop_worker_thread(self):
        """
        Stops the worker thread after the 'turn off cooler' button is
        pushed.
        """
        print("stop_worker_thread called")
        if self.warmupThread:
            print("warmupThread exists, starting...")
            self.warmupThread.s_init()
            self.warmupThread.start()
        if self.workerThread:
            print("ending worker thread.")
            self.workerThread.end_this_thread()
            self.workerThread.wait()
            self.workerThread = None
        else:
            self.shut_down_if_necessary()
        # self.stop()

    @pyqtSlot()
    def shut_down_if_necessary(self):
        print("shut_down_if_necessary called")
        if self.want_to_shut_down == True:
            self.stop()  # stops any running threads
            self.warmupThread = None
            self.workerThread = None

            if self.stderrThread:
                self.stderrThread.end_this_thread()
                self.stderrThread.wait()
                self.stderrThread = None

            self.model.__del__()  # this is safe; it will raise the
            # temperature first.
            quit()
        else:
            pass

    def shut_down(self):
        print('Shut Down button pressed.')
        self.want_to_shut_down = True
        self.stop_worker_thread()
        # ^ this then calls shut_down_if_necessary

    @pyqtSlot(str)
    def update_stderr(self, current_stderr):
        self.error_window.clear()
        time.sleep(0.01)  # just so you can see the window updating
        self.error_window.setText(current_stderr)

    @pyqtSlot(int)
    def update_temp(self, temperature):
        self.display_temperature = temperature

    @pyqtSlot()
    def data_acqusition_done(self):  # maybe put the 'save' function here?
        time.sleep(1)
        self.workerThread.end_this_thread()
        self.workerThread.wait()
        self.workerThread = None
        print("acquisition done.")

    @pyqtSlot(str)
    def acknowledge_signal(self, arg=''):
        print("got signal with argument", arg)

    def selectFile(self):
        self.path_name.setText(
            QtWidgets.QFileDialog.getExistingDirectory())
        self.data_path = self.path_name.text()
        print("data path now is", self.data_path)
        self.update_model()

    def select_calibration_spectrum_file(self):
        self.calibration_spectrum_path.setText(
            QtWidgets.QFileDialog.getOpenFileName())
        self.calibration_spectrum_file = \
            self.calibration_spectrum_path.text()
        self.update_model()
        self.plot_calibration_data(
            self.calibration_spectrum_file)

    def select_background_file(self):
        self.background_path.setText(
            QtWidgets.QFileDialog.getOpenFileName())
        self.background_file = \
            self.background_path.text()
        print("background noise file is now", self.background_file)
        self.update_model()

    def select_display_file(self, add=False):
        """
        Selects a file to display on the "Display data" tab.
        add: if True, multiple plots are overlaid.
        """
        self.display_path.setText(
            QtWidgets.QFileDialog.getOpenFileName())
        if os.path.isfile(str(self.display_path.text())):
            self.display_file = self.display_path.text()
            self.update_model()
            print("DISPLAY FILE", self.display_file)
            self.update_plot(which_plot='display', add=add)
            print("DISPLAY FILE", self.display_file)
        else:
            self.display_path.undo()

    def select_cal_file(self):
        """
        Selects a .cal file to convert pix to wavelength.
        """
        temp_cal_path = QtWidgets.QFileDialog.getOpenFileName()
        self.cal_file = str(temp_cal_path)
        self.load_cal_file(self.cal_file, which_plot='main')
        self.update_model()
        self.update_plot(which_plot='main')

    def select_cal_file_display(self):
        """
        Selects a .cal file to convert pix to wavelength.
        This applies the function to the display panel.
        """
        self.cal_path.setText(
            QtWidgets.QFileDialog.getOpenFileName())
        self.cal_file_display = str(self.cal_path.text())
        self.load_cal_file(
            self.cal_file_display, which_plot='display')
        self.update_model()
        self.update_plot(which_plot='display')

    def load_cal_file(self, cal_filename, which_plot):
        """
        which_plot: 'main' or 'display'
        """
        assert os.path.isfile(cal_filename)

        if cal_filename.endswith('.cal'):

            print("loading cal", cal_filename)

            with open(cal_filename, 'r') as cal_file:
                for line in cal_file:
                    print("cal file contents:", line)
                    _m, _b = [float(n) for n in line.split()]

            def loaded_function(x):
                return _m * x + _b

            if which_plot == 'main':
                self.pix_to_wavelength = loaded_function
                print("main calbration file loaded.")

                try:
                    assert os.path.isfile(self.current_plot_file)
                    self.graph_box.canvas.ax1.set_xlabel(
                        'Wavelength (nm)')
                    self.update_plot(which_plot='main')
                except TypeError:
                    print("$$")
                    pass

            elif which_plot == 'display':
                self.pix_to_wavelength_display = loaded_function
                print("display calbration file loaded.")

                try:
                    assert os.path.isfile(self.current_display_file)
                    self.display_box.canvas.ax1.set_xlabel(
                        'Wavelength (nm)')
                    self.update_plot(which_plot='display')
                except TypeError:
                    print("$")
                    pass

        else:
            print("file must end in .cal")
            self.cal_path.undo()

    def collect_data(self):
        """
        Initialize the camera and collect data.
        """
        print(self.data_path + self.data_name + '.txt')
        # self.stop() # kills threads in case some are running before execution

        if (os.path.isfile(self.data_path + '\\' + self.data_name + '.txt')) and not (
                str(self.data_name).startswith("throwaway")):
            choice = QtWidgets.QMessageBox.question(
                self, 'Window', "File already exists! Overwrite?",
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No)

            if choice == QtWidgets.QMessageBox.Yes:
                self.update_model()
                self.get_data_button_has_been_pushed = True
                self.start()
            else:
                pass

        else:
            self.update_model()
            self.get_data_button_has_been_pushed = True
            self.start()

    def end_stepper_thread(self):
        print("ending stepper thread.")
        self.move_motor_up.setEnabled(True)
        self.move_motor_down.setEnabled(True)
        self.stepperThread.quit()
        self.stepperThread.wait()

    def freeze_stepper_buttons(self):
        self.move_motor_up.setEnabled(False)
        self.move_motor_down.setEnabled(False)

    # this code is sort of redundant. consider using __getattr__ instead
    def change_trigger_type(self):  # triggers trigger @setter
        self.trigger_type = self.trigger_selector.currentIndex()

    def change_temperature(self):  # triggers temperature @setter
        self.temperature = self.temperature_box.text()
        print("DEBUG: good, temperature is now", self.temperature_box.text())

    def change_exposure_time(self):  # triggers exposure @setter
        self.exposure_time = self.exposure_time_box.text()
        # ^exposure_time_box.text() gets passed as 'value' to setter
        print("DEBUG: good, exposure time is now", \
              self.exposure_time_box.text())

    def change_step_size(self):  # changes stepper step size
        self.step_size = self.step_size_box.text()
        print("DEBUG: good, step size is now", \
              self.step_size_box.text())

    def change_data_name(self):
        self.data_name = self.data_name_box.text()
        print("DEBUG: good, data name is now", self.data_name_box.text())

    def change_checked_type(self):
        self.continue_taking_data = \
            self.continue_taking_data_box.isChecked()
        print("DEBUG: good, contnue_taking_data is now", \
              self.continue_taking_data_box.isChecked())

    @pyqtSlot('PyQt_PyObject')
    def plot_max_data(
            self, path_to_file, display=False, add_plots=False):
        """
        Takes the mean of each column and plots that.
        Input:
            path_to_file: path to .txt file of data
            display: if True, plots the data on the 'Display' tab
            add_plots: if True, stacks plots
        Output:
            y_vals: array of mean value of each row of image.
        Also triggers the function that shows the .bmp file.
        """
        print("plotting file", path_to_file)
        if display == False:
            box = self.graph_box
        else:
            box = self.display_box

        data = []
        with open(path_to_file) as image:
            for line in image:
                if not line.startswith('#'):
                    data.append(float(line))
                else:
                    print(line)

        background = []
        if self.background_file:
            assert os.path.isfile(self.background_file)
            print("opening background file...")

            with open(self.background_file) as bg_image:
                for line in bg_image:
                    if not line.startswith('#'):
                        background.append(float(line))
                    else:
                        print(line)

            bg = np.reshape(
                np.array(background),
                (self.model._height, self.model._width)
            )
        else:
            print("no background file.")

        if add_plots == False:
            box.canvas.ax1.clear()
        else:
            box.canvas.ax2.cla()
            print("NOT CLEARING GRAPH")

        # box.canvas.ax1.margins(x=0)
        # ^so that the spectrum image lines up with the graph

        actual_height = len(data) // self.model._width
        print("Calculated height:", actual_height)
        im = np.reshape(
            np.array(data), (self.model._height, self.model._width))
        if self.background_file:
            assert im.shape == bg.shape
            im = np.subtract(im, bg)

        print("Shape:", im.shape)

        # remove outliers
        print("Width:", self.model._width)
        print("Height:", self.model._height)
        # y_vals = [sum(im[i,:]) for i in range(self.model._width)]
        # y_vals = [max(list(im[i,:])) for i in range(self.model._width)]
        print("PLOTTING:", self.y_min, self.y_max)
        if display == True:
            y_vals = [np.mean(
                im[self.y_min_display:self.y_max_display, i]
            ) for i in range(self.model._width)]
            x_vals = [
                self.pix_to_wavelength_display(i) for i in range(
                    self.model._width)
            ]
            self.display_data = (x_vals, y_vals)
            self.display_im = im
        else:
            x_vals = [
                self.pix_to_wavelength(i) for i in range(
                    self.model._width)
            ]
            y_vals = [np.mean(
                im[self.y_min:self.y_max, i]
            ) for i in range(self.model._width)]

        y_vals = y_vals[::-1]  # reverses list

        # y_vals = im.max(axis=1)
        std = np.std(y_vals)
        mean = np.mean(y_vals)
        m = 6
        print("Mean:", mean, "std:", std, "m*std", m * std)

        list_max_vals = False

        if list_max_vals == True:
            self.list_max_vals(x_vals, y_vals)

        coords = list(zip(x_vals, y_vals))
        outliers = []

        for coord in coords:
            if (abs(coord[1] - mean) > m * std):
                coords.remove(coord)
                outliers.append(coord)
                # print "Found outlier", coord

        if display == False:
            # ^i.e. if we're not plotting on the display tab
            box.canvas.ax1.plot(*list(zip(*coords)),
                                color='green', linewidth=0.4)
        else:
            box.canvas.ax1.plot(
                *list(zip(*coords)), linewidth=0.4,
                label=str(path_to_file))
            box.canvas.ax1.legend(loc='upper left')

        if outliers:
            #    self.graph_box.canvas.ax.scatter(*zip(*outliers),
            #        c='red', marker='x')
            print("Outliers.")
        else:
            print("No outliers.")

        # self.graph_box.canvas.ax.set_xlim((0,1024))
        box.canvas.ax1.set_xlabel('Wavelength (nm)')
        box.canvas.ax1.set_ylabel('Count (mean)')

        box.canvas.draw()

        # this function also triggers the imshow function:
        self.show_bmp_image(
            str(path_to_file).split('.')[0] + '.bmp', display=display)

        print("\tminimum y =", self.y_min_display)
        print("\tmaximum y =", self.y_max_display)
        return y_vals

    def plot_calibration_data(self, path_to_file):
        """
        Plots a spectrum (the spectrum from the Oriel neon calibration
            lamp) and shows the locations of each peak.

        Inputs:
            path_to_file: path to .txt file.
        """
        assert os.path.isfile(path_to_file)
        self.calibration_graph.canvas.ax1.cla()

        image_array = self.plot_max_data(
            path_to_file)  # plots on the main page too

        x_vals = list(range(len(image_array)))
        self.calibration_graph.canvas.ax1.plot(x_vals, image_array,
                                               'k-', linewidth=0.4)
        m_list = calibrate.get_maxima_locations(
            np.array(image_array))  # max list

        for i_x, i_y in zip(m_list, [image_array[j] for j in m_list]):
            self.calibration_graph.canvas.ax1.text(
                i_x, i_y, '{}'.format(i_x), fontsize=6, color='red')

        self.calibration_graph.canvas.draw()

    def list_max_values(self, x_vals, y_vals):
        std = np.std(y_vals)
        mean = np.mean(y_vals)
        m = 6

        maxima_locations = list(
            argrelextrema(np.array(y_vals), np.greater, order=6))[0]

        maxima_locations = [
            loc for loc in maxima_locations if not \
                (y_vals[loc] < (min(y_vals) + std * 0.1))]

        self.graph_box.canvas.ax1.scatter(
            maxima_locations, [y_vals[i] for i in maxima_locations],
            c='red', s=0.4, marker='x')

        print("Maxima:")
        for i in maxima_locations:
            print("Pixel:", x_vals[i])
            print("Wavelength:", self.get_wavelength(x_vals[i]))
            print("Value:", y_vals[i])
            print("---")

        peak_differences = np.diff(maxima_locations)
        peak_differences_normalized = [
            i / max(peak_differences) for i in peak_differences]

    def show_bmp_image(self, path_to_file, display=False):
        """
        Shows a bmp image.
        """
        if display == False:
            box = self.graph_box
        else:
            box = self.display_box

        xtick_locations = list(range(0, self.model._width,
                                     int(self.model._width / 10)))
        box.canvas.ax2.clear()
        # self.graph_box.canvas.ax2.set_xticks(xtick_locations)
        # self.graph_box.canvas.ax2.set_xticklabels(
        #    [round(self.get_wavelength(i), 2) for i in xtick_locations])
        # self.graph_box.canvas.ax1.xaxis.set_tick_params(rotation=45)

        img = Image.open(path_to_file)
        img = img.rotate(270, expand=1)
        self.my_imshow = box.canvas.ax2.imshow(
            img, cmap='gray', vmin=0, vmax=255, origin='lower',
            interpolation='none')
        if display == False:
            self.my_imshow.set_extent(
                extent=[self.pix_to_wavelength(0),
                        self.pix_to_wavelength(1023),
                        0, 127]
            )
        elif display == True:
            self.my_imshow.set_extent(
                extent=[self.pix_to_wavelength_display(0),
                        self.pix_to_wavelength_display(1023),
                        0, 127]
            )
            if self.y_min_display and self.y_max_display:
                box.canvas.ax2.axhline(
                    self.y_min_display, color='red', linewidth=0.4)
                box.canvas.ax2.axhline(
                    self.y_max_display, color='red', linewidth=0.4)

        box.canvas.ax2.set_aspect('auto')
        box.canvas.draw()

    def update_model(self):
        """
        pass any user input onto the model and eventually the camera.
        """
        # eventually I can just modify the internal parameters
        # in the AndorIdus class but I don't want to do that yet
        # self.model is an instance of the AndorIdus class
        self.model.set_exposure_time = self.exposure_time
        self.model.set_temperature = self.temperature
        self.model.set_trigger_type = self.trigger_type
        self.model.set_full_data_path = self.data_path + '\\' + self.data_name
        self.model._continue_taking_data = self.continue_taking_data

    def update_ui_from_model(self):
        """
        passes information from model back to the UI
        (e.g., cooling status light)
        """
        self.cooler_on = self.model._cooler_on
        # self.display_temperature = self.model._temperature
        self.exposure_led.setValue(self.model._currently_taking_data)
        if self.should_i_plot == True:
            self.plot_max_data(self.model.true_path)
        # ^should put this somewhere else in next version

    def update_plot(self, which_plot, add=False):
        """
        Updates the main or display plot, usually after the x axis
        has been calibrated.

        which_plot = 'main' or 'display'
        """
        if which_plot == 'main':
            self.plot_max_data(self.current_plot_file)
            print("updating main plot")
        elif which_plot == 'display':
            self.plot_max_data(
                self.current_display_file, display=True,
                add_plots=add)
            print("updating display plot")
        else:
            print("not a valid update_plot option.")
    @pyqtSlot(bool)
    def toggle_parameter_lock(self, val=0):
        """
        Locks/unlocks the parameter input boxes so that the user can't
        change anything while the camera is running.
        0 = unlocked
        1 = locked
        """
        if val == 0:
            print("Unlocking...")
        if val == 1:
            print("Locking...")

        self.temperature_box.setReadOnly(val)
        self.exposure_time_box.setReadOnly(val)
        self.trigger_selector.setEnabled(not val)
        self.data_name_box.setReadOnly(val)
        self.stop_cooler_button.setEnabled(not val)
        self.start_button.setEnabled(not val)
        self.continue_taking_data_box.setEnabled(not val)

    # saving functions (originally in AndorIdus class)

    @pyqtSlot('PyQt_PyObject',int,int,'PyQt_PyObject')
    def save_as_bmp(self, data_path, width, height, image_array):
        '''
        Save the most recent acquired image as a bitmap

        Input:
            data_path (string) : Filename to save to
            width, height: width and height of image
            image_array: 1D array of image data (if the image is
                127 x 1024 pixels, image_array will have 130048
                elements.

        Output:
            None
        '''
        print("saving as bmp")
        im = Image.new("RGB", (height, width), "white")
        pix = im.load()
        max_intensity = max(image_array)
        min_intensity = min(image_array)
        print(max_intensity, min_intensity)

        print("ready for save loop")
        for i in range(len(image_array)):
            (row, col) = divmod(i, width)
            picvalue = int(round((image_array[i] - min_intensity) * 255.0 /
                                 (max_intensity - min_intensity)))
            pix[row, col] = (picvalue, picvalue, picvalue)

        print(pix[1, 1])
        print(pix[9, 100])
        data_path = str(data_path)
        im.save(data_path, "BMP")
        err = "BMP saved."
        print(err)
        self.model.current_output = err

    @pyqtSlot('PyQt_PyObject','PyQt_PyObject')
    def save_as_txt(self, path, image_array):
        '''
        Save the most recent acquired image as txt

        Input:
            path (string) : Filename to save to

        Output:
            None
        '''
        self._should_i_plot = True
        with open(path, 'w') as filename:
            print("saving as", path)

            filename.write(
                "# taken with temp={0} C, exposure={1} s, mode={2}\n".format(
                    self.temperature, self.exposure_time,
                    self.model.acquisition_mode))
            filename.write(
                "# taken on {0}\n".format(str(datetime.datetime.now())))
            for line in image_array:
                filename.write("%g\n" % line)

        err = ".txt file saved."
        print(err)
        self.model.current_output = err

    def closeEvent(self, event):
        """
        Overrides the closeEvent qt function so that the camera
        shuts down before the user presses the red 'x' button.
        """
        print("x button clicked.")
        choice = QtWidgets.QMessageBox.question(
            self, 'Window', "Do you actually want to quit?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No)

        if choice == QtWidgets.QMessageBox.Yes:
            self.shut_down()
            event.accept()
        else:
            event.ignore()
