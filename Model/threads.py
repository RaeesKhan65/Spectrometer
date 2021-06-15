import sys
import time
import os
import numpy as np
from PyQt5.QtCore import pyqtSlot,pyqtSignal
from PyQt5 import QtCore, QtWidgets, uic
import traceback

class WorkerThread(QtCore.QThread):
    """
    This thread does the work of taking the picture. The AndorIdus() class
    instance is created in the main view and passed as a parameter to this
    thread. It includes a 'running' variable to specify whether the camera is
    doing work.
    """
    dummy = pyqtSignal(str, name='dummy')
    toggle_parameter_lock = pyqtSignal(bool, name= "toggle_parameter_lock")
    save_as_bmp = pyqtSignal('PyQt_PyObject', int, int, 'PyQt_PyObject', name = "save_as_bmp")
    save_as_txt = pyqtSignal('PyQt_PyObject', 'PyQt_PyObject', name = 'save_as_txt')
    make_plot = pyqtSignal('PyQt_PyObject',name = 'make_plot' )
    data_acquistion_done = pyqtSignal(str, name = 'data_acquistion_done')
    Stop = pyqtSignal(str, name='Stop')

    def __init__(self, camera):
        QtCore.QThread.__init__(self, None)
        self.running = False
        self.camera = camera
        self._temperature = camera._temperature

    def s_init(self):
        print("s_init (workerthread)")
        self.running = True

    def run(self):
        try:
            self.running = True
            #self.emit(QtCore.SIGNAL('Dummy'), 'Good')
            self.dummy.emit("Good")
            #self.emit(QtCore.SIGNAL('toggle_parameter_lock(bool)'), 1)
            self.toggle_parameter_lock.emit(1)
            # ^ changes the parameters to 'locked' during acqusition
            self.camera.implement_parameters()
            self.msleep(500)
            if self.camera.emergency_stop_cooling != True:
            #    self.emit(QtCore.SIGNAL(
             #           'save_as_bmp(PyQt_PyObject, int, int, PyQt_PyObject)'
              #      ),
              #      self.camera.set_full_data_path + '.bmp',
               #     self.camera._width, self.camera._height,
                #    self.camera._imageArray)

                self.save_as_bmp.emit(self.camera.set_full_data_path + '.bmp',
                                      self.camera._width, self.camera._height,
                                      self.camera._imageArray)

               # self.emit(QtCore.SIGNAL(
                #        'save_as_txt(PyQt_PyObject, PyQt_PyObject)'
                #    ),
                 #   self.camera.set_full_data_path + '.txt',
                 #   self.camera._imageArray)

                self.save_as_txt.emit(self.camera.set_full_data_path + '.txt', self.camera._imageArray)


              #  self.emit(QtCore.SIGNAL('make_plot(PyQt_PyObject)'),
               #     self.camera.set_full_data_path + '.txt')

                self.make_plot.emit(self.camera.set_full_data_path + '.txt')

              #  self.emit(QtCore.SIGNAL('toggle_parameter_lock(bool)'), 0)

                self.toggle_parameter_lock.emit(0)

        except BaseException as e:
            print(e.args[0])
           # self.emit(QtCore.SIGNAL('data_acquistion_done'))
            self.data_acquistion_done.emit('data_acquistion_done')
            self.msleep(500)
            self.running = False
            sys.stderr.write(traceback.format_exc())

    def end_this_thread(self):
        self.running = False

class WarmupThread(QtCore.QThread):
    """
    This thread warms the camera up.
    """

    toggle_parameter_lock = pyqtSignal(bool, name="toggle_parameter_lock")
    warming = pyqtSignal(str, name = 'warming')
    warming_done = pyqtSignal(str, name='warming_done')

    def __init__(self, camera):
        QtCore.QThread.__init__(self,None)
        self.running = False
        self.camera = camera

    def s_init(self):
        print("s_init (warmup)")
        self.running = True

    def run(self):
        try:
            print("warming up from warmupThread")
            self.running = True
            #self.emit(QtCore.SIGNAL('warming'))
            self.warming.emit('warming')
            try:
                if self.camera._cooler_on:
                    self.camera.emergency_stop_cooling = True
            except BaseException as e:
                print("inner error")
                print(e.args[0])
                pass
            # ^this essentially interrupts the worker thread.
            print("took care of that")
          #  self.emit(QtCore.SIGNAL('toggle_parameter_lock(bool)'), 1)
            self.toggle_parameter_lock.emit(1)
            self.msleep(500)
            self.camera.WarmUp()
           # self.emit(QtCore.SIGNAL('toggle_parameter_lock(bool)'), 0)
            self.toggle_parameter_lock.emit(0)
            self.msleep(500)
            #self.emit(QtCore.SIGNAL('warming_done'))
            self.warming.emit('warming_done')
            self.msleep(500)
            print("warming_done signal")
        except BaseException as e:
            print("warmupThread error")
            print(e.message)
            self.running = False
            sys.stderr.write(traceback.format_exc())

    def end_this_thread(self):
        print("ending warmup thread.")
        self.running = False


class TemperatureThread(QtCore.QThread):
    """
    Thread that automatically updates the temperature.
    This is now incorporated in the StderrThread.
    """
    update_temp = pyqtSignal(int, name= 'update_temp')

    def __init__(self, camera):
        QtCore.QThread.__init__(self, None)
        self.running = False
        self.camera = camera

    def s_init(self):
        print("s_init (temp)")
        self.running = True

    def run(self):
        try:
            self.running = True
            while self.running:
                print("temp thread.")
                self.camera.GetTemperature()
            #    self.emit(
            #        QtCore.SIGNAL('update_temp(int)'),
            #        self.camera._temperature)
                self.update_temp.emit(self.camera._temperature)
                self.msleep(5000)
        except:
            self.running = False
            sys.stderr.write(traceback.format_exc())

    def end_this_thread(self):
        # print "Turning off"
        self.running = False

class StderrThread(QtCore.QThread):
    """
    This thread updates the display box in the GUI with the stderr.
    Also updates the temperature display.
    """
    update_stderr = pyqtSignal(str, name = 'update_stderr')
    update_temp = pyqtSignal(int, name = 'update_temp')

    def __init__(self, camera):
        QtCore.QThread.__init__(self,None)
        self.running = True
        self.camera = camera

    def s_init(self):
        print("s_init (stderrthread)")
        self.running = True

    def run(self):
        try:
            self.running = True
            while self.running:
             #   self.emit(
             #       QtCore.SIGNAL('update_stderr'),
             #       self.camera.current_output)
                self.update_stderr.emit(self.camera.current_output)
                self.camera.GetTemperature()
             #   self.emit(
             #       QtCore.SIGNAL('update_temp(int)'),
             #       self.camera._temperature)
                self.update_temp.emit(self.camera._temperature)
                self.msleep(1000)
        except:
            sys.stderr.write(traceback.format_exc())

    def end_this_thread(self):
        # print "Turning off"
        self.running = False

class MoveMotorThread(QtCore.QThread):
    done_stepping = pyqtSignal(str, name = 'done_stepping')
    stepping = pyqtSignal(str, name='stepping')

    def __init__(self, stepper, step_size, direction):
        QtCore.QThread.__init__(self,None)
        print("starting stepper thread.")
        self.stepper = stepper
        self.step_size = step_size
        self.direction = direction

    def run(self):
        print("running")
        try:
            self.stepper.step(self.step_size, self.direction)
           # self.emit(QtCore.SIGNAL('done_stepping'))
            self.done_stepping.emit('done_stepping')
        except BaseException as e:
            print(e.args[0])
            print("Error running stepperThread.")
         #   self.emit(QtCore.SIGNAL('done_stepping'))
            self.done_stepping.emit('done_stepping')

class DummyWorkerThread(QtCore.QThread):
    """
    For testing.
    """
    dummy = pyqtSignal(str, name = 'dummy')
    stop = pyqtSignal(str, name = 'stop')
    def __init__(self, camera):
        QtCore.QThread.__init__(self, None)
        self.running = False
        self.camera = camera

    def s_init(self):
        print("s_init")
        self.running = True

    def run(self):
        try:
            self.running = True
            #self.emit(QtCore.SIGNAL('Dummy'), 'Good')
            self.dummy.emit('Good')
            self.camera.implement_parameters()
            print("---")
            self.camera.WarmUp()
            self.msleep(500)
            #self.emit(QtCore.SIGNAL('Stop'))
            self.stop.emit('stop')
        except:
            self.running = False
            sys.stderr.write(traceback.format_exc())

    def end_this_thread(self):
        #print "Turning off"
        self.running = False