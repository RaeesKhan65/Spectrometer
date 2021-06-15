from ctypes import windll, c_int, c_char, byref, c_long, \
    pointer, c_float, c_char_p, cdll
from PIL import Image
from matplotlib import pyplot as plt
from PyQt5 import QtWidgets
from PyQt5.QtCore import QThread, pyqtSignal, QObject
import sys
import time
import datetime
import inspect
import platform
#import visa
import pyvisa as visa
import os
import numpy as np

debugging = False # set to true to get verbose output

class Stepper(object):
    """
    Class which provides basic functions to control the stepper motor.
    """
    # set up communication with Arduino
    def __init__(self):
        try:
            self.rm = visa.ResourceManager()
            self.visas = self.rm.list_resources()
            self.workable_visas = [
                v for v in self.visas if v.startswith(("COM", "ASRL"))]
            print("usable visas: ", self.workable_visas)

            #self.my_visa = self.workable_visas[-1]
            self.my_visa = "ASRL8::INSTR"
            # ^assume the first one is correct


            print("using", self.workable_visas[-1])

            self.inst = self.rm.open_resource(self.my_visa)
            print("IDN? response:", self.inst.query("*IDN?"))
        except BaseException as e:
            print("No communication with Arduino.")
            print(e.args[0])
            pass

    def step(self, number_of_steps, direction):
        """
        Steps the motor. 'U' => up, 'D' => down
        """
        if len(str(number_of_steps)) != 5:
            pass # the length should be constrained by the model.
        if direction in ['U', 'D']:
            command = "R" + number_of_steps + direction
        print(command)
        self.inst.write(command)
        time.sleep(1)
        response = self.inst.read()
        if debugging == True:
            print("Response:", response)


class AndorIdus(object):
    """
    Andor class which is meant to provide the Python version of the same
    functions that are defined in the Andor's SDK. Extensive documentation
    on the functions used and error codes can be
    found in the Andor SDK Users Guide
    """

    def __init__(self):
        '''
        Loads and initializes the hardware driver.
        Initializes local parameters
        '''

        # Check operating system and load library
        if platform.system() == "Linux":
            dllname = "/usr/local/lib/libandor.so"
            self._dll = cdll.LoadLibrary(dllname)
        elif platform.system() == "Windows":
            print("Loading Windows library...", end=' ')
            dllname = \
                "C:\\Program Files\\Andor SOLIS\\Drivers\\atmcd64d.dll"
            self._dll = windll.LoadLibrary(dllname)
            print("done.")
        else:
            print("Cannot detect operating system, wil now stop")
            raise ValueError

        # Initialize the device
        print("Initializing...", end=' ')
        tekst = c_char()
        error = self._dll.Initialize(byref(tekst))
        print("%s" % (ERROR_CODE[error]))

        cw = c_int()
        ch = c_int()
        self._dll.GetDetector(byref(cw), byref(ch))


        # Initiate parameters

        # model variables, to be set by the view/controller
        self.set_trigger_type = None  # 0 = internal; 1 = external
        self.set_temperature = None  # degrees celcius
        self.set_exposure_time = None  # seconds
        self.set_data_name = ""
        self.set_full_data_path = ""
        self.true_path = ""  # ugly
        self.set_aquisition_mode = ""
        self.acquisition_mode = 'Image'  # FVB or Image

        self.current_output = ""  # stderr ouput
        self._currently_taking_data = False
        self._continue_taking_data = False  # continue data acquisition?
        self.emergency_stop_cooling = False
        # ^set to true when cooler is interrupted by Shut Down.
        #  for each cycle of CoolDown, the camera checks to see if
        #  this variable is True.

        self._view_funcs = []  # functions to execute from the view

        self._width = cw.value
        self._height = ch.value
        self._temperature = None
        self._temp_min = None  # minumum temp camera can be cooled to.
        self._temp_max = None  # ^(see GetTemperatureRange)
        self._set_T = None
        self._gain = None
        self._gainRange = None
        self._status = ERROR_CODE[error]
        self._verbosity = True
        self._preampgain = None
        self._channel = None
        self._outamp = None
        self._hsspeed = None
        self._vsspeed = None
        self._serial = None
        self._exposure = None
        self._accumulate = None
        self._kinetic = None
        self._bitDepths = []
        self._preAmpGain = []
        self._VSSpeeds = []
        self._noGains = None
        self._imageArray = []
        self._noVSSpeeds = None
        self._HSSpeeds = []
        self._noADChannels = None
        self._noHSSpeeds = None
        self._ReadMode = None
        self._cooler_on = False

        if ERROR_CODE[error] is not "DRV_SUCCESS":
            self._width = 1024
            self._height = 127
            print("IUHWIEURHIWUEHR")

    def __del__(self):
        print("__del__")
        target_temperature = -20  # degrees C; don't shut camera down
        # until the temperature is over -20

        if self._temperature < target_temperature and \
                self._temperature != None:

            if self.GetTemperature() not in [
                "DRV_TEMP_OFF", "DRV_NOT_INITIALIZED"]:
                print("turning cooler off")
                self.CoolerOFF()

            print("warming")
            while self._temperature < target_temperature:
                self.GetTemperature()
                time.sleep(1)

        error = self._dll.ShutDown()
        self._Verbose(ERROR_CODE[error])

    ##### some view-related functions #####

    def implement_parameters(self):
        """
        The user can change the internal variables of the AndorIdus class.
        This function actually passes these changes on to the camera.
        """
        PreAmpGain = 0
        self.SetExposureTime(self.set_exposure_time)
        self.SetTriggerMode(self.set_trigger_type)
        if self.acquisition_mode == 'FVB':
            print('setting FVB')
            self.SetSingleFVB()
        elif self.acquisition_mode == 'Image':
            print('setting single image')
            self.SetSingleImage()
        else:
            print("acquisition_mode doesn't exist.")
            self.__del__()
        self.SetTriggerMode(0)
        self.SetShutter(1, 1, 0, 0)
        self.SetPreAmpGain(PreAmpGain)
        print("Collecting data.")
        print("Exposure time is", self.set_exposure_time)
        print("Desired temperature is", self.set_temperature)
        self.CoolDown(self.set_temperature)
        if self.acquisition_mode == 'FVB':
            self.FVBCapture()
        elif self.acquisition_mode == 'Image':
            self.ImageCapture()
        self.announce_update_to_model()  # hopefully this makes a graph
        if self._continue_taking_data == True:
            print("Still taking data.")
            pass
        else:
            self.WarmUp()

    def announce_update_to_model(self):
        """
        I wrote this function when it seemed like I'd have a lot of
        functions to pass to the model, but this probably isn't
        necessary.
        """
        for func in self._view_funcs:
            func()

    #########################################

    def LINE(self, back=0):
        '''
        Return line of statement in code

        Input:
            back (int)   : The number of positions to move
                           up in the calling stack (default=0)

        Output:
            (string)     : The requested information
        '''
        return sys._getframe(back + 1).f_lineno

    def FILE(self, back=0):
        '''
        Return filename of source code

        Input:
            back (int)   : The number of positions to move
                           up in the calling stack (default=0)

        Output:
            (string)     : The requested information
        '''
        return sys._getframe(back + 1).f_code.co_filename

    def FUNC(self, back=0):
        '''
        Return function name

        Input:
            back (int)   : The number of positions to move
                           up in the calling stack (default=0)

        Output:
            (string)     : The requested information
        '''
        return sys._getframe(back + 1).f_code.co_name

    def WHERE(self, back=0):
        '''
        Return information of location of calling function

        Input:
            back (int)   : The number of positions to move
                           up in the calling stack (default=0)

        Output:
            (string)     : The requested information
        '''
        frame = sys._getframe(back + 1)
        return "%s/%s %s()" % (os.path.basename(frame.f_code.co_filename),
                               frame.f_lineno, frame.f_code.co_name)

    def _Verbose(self, error):
        '''
        Reports all error codes to stdout if self._verbosity=True

        Input:
            error (string)  : The string resulted from the error code
            name (string)   : The name of the function calling the device

        Output:
            None
        '''
        self.current_output = "[%s]: %s" % (self.FUNC(1), error)
        if self._verbosity is True:
            print("[%s]: %s" % (self.FUNC(1), error))

    def SetVerbose(self, state=True):
        '''
        Enable / disable printing error codes to stdout

        Input:
            state (bool)  : toggle verbosity, default=True

        Output:
            None
        '''
        self._verbosity = state

    # Get Camera properties

    def GetCameraSerialNumber(self):
        '''
        Returns the serial number of the camera

        Input:
            None

        Output:
            (int) : Serial number of the camera
        '''
        serial = c_int()
        error = self._dll.GetCameraSerialNumber(byref(serial))
        self._serial = serial.value
        self._Verbose(ERROR_CODE[error])
        return self._serial

    def GetNumberHSSpeeds(self):
        '''
        Returns the number of HS speeds

        Input:
            None

        Output:
            (int) : the number of HS speeds
        '''
        noHSSpeeds = c_int()
        error = self._dll.GetNumberHSSpeeds(self._channel, self._outamp,
                                            byref(noHSSpeeds))
        self._noHSSpeeds = noHSSpeeds.value
        self._Verbose(ERROR_CODE[error])
        return self._noHSSpeeds

    def GetNumberVSSpeeds(self):
        '''
        Returns the number of VS speeds

        Input:
            None

        Output:
            (int) : the number of VS speeds
        '''
        noVSSpeeds = c_int()
        error = self._dll.GetNumberVSSpeeds(byref(noVSSpeeds))
        self._noVSSpeeds = noVSSpeeds.value
        self._Verbose(ERROR_CODE[error])
        return self._noVSSpeeds

    # Cooler and temperature
    def CoolerON(self):
        '''
        Switches the cooler on

        Input:
            None

        Output:
            None
        '''
        error = self._dll.CoolerON()
        self._Verbose(ERROR_CODE[error])
        self._cooler_on = True
        self.announce_update_to_model()

    def CoolerOFF(self):
        '''
        Switches the cooler off

        Input:
            None

        Output:
            None
        '''
        error = self._dll.CoolerOFF()
        self._Verbose(ERROR_CODE[error])
        self._cooler_on = False
        self.announce_update_to_model()

    def SetCoolerMode(self, mode):
        '''
        Set the cooler mode

        Input:
            mode (int) : cooler modus

        Output:
            None
        '''
        error = self._dll.SetCoolerMode(mode)
        self._Verbose(ERROR_CODE[error])

    def IsCoolerOn(self):
        '''
        Returns cooler status

        Input:
            None

        Output:
            (int) : Cooler status
        '''
        iCoolerStatus = c_int()
        error = self._dll.IsCoolerOn(byref(iCoolerStatus))
        self._Verbose(ERROR_CODE[error])
        return iCoolerStatus.value

    def GetTemperatureRange(self):
        '''
        Returns the range of valid temperatures.
        '''
        temp_min = c_int()
        temp_max = c_int()
        error = rc = self._dll.GetTemperatureRange(byref(temp_min), byref(temp_max))
        self._Verbose(ERROR_CODE[error])
        print("Valid temperature range is", temp_min.value, "to", temp_max.value, "degrees celsius.")
        self._temp_min = temp_min.value
        self._temp_max = temp_max.value
        return (temp_min.value, temp_max.value)

    def GetTemperature(self):
        '''
        Returns the temperature in degrees Celcius

        Input:
            None

        Output:
            (int) : temperature in degrees Celcius
        '''
        ctemperature = c_int()
        error = self._dll.GetTemperature(byref(ctemperature))
        self._temperature = ctemperature.value
        self._Verbose(ERROR_CODE[error])
        # print "Temperature is: %g [Set T: %g]" \
        #    % (self._temperature, self._set_T)
        self.announce_update_to_model()
        return ERROR_CODE[error]

    def SetTemperature(self, temperature):  # Fixme:, see if this works
        '''
        Set the working temperature of the camera

        Input:
            temparature (int) : temperature in degrees Celcius

        Output:
            None
        '''
        #        ctemperature = c_int(temperature)
        error = self._dll.SetTemperature(temperature)
        self._set_T = temperature
        self._Verbose(ERROR_CODE[error])
        self.announce_update_to_model()

    ###### Single Parameters Set ######
    def SetAccumulationCycleTime(self, time_):
        '''
        Set the accumulation cycle time

        Input:
            time_ (float) : the accumulation cycle time in seconds

        Output:
            None
        '''
        error = self._dll.SetAccumulationCycleTime(c_float(time_))
        self._Verbose(ERROR_CODE[error])

    def SetAcquisitionMode(self, mode):
        '''
        Set the acquisition mode of the camera

        Input:
            mode (int) : acquisition mode

        Output:
            None
        '''
        error = self._dll.SetAcquisitionMode(mode)
        self._Verbose(ERROR_CODE[error])

    def SetADChannel(self, index):
        '''
        Set the A-D channel for acquisition

        Input:
            index (int) : AD channel

        Output:
            None
        '''
        error = self._dll.SetADChannel(index)
        self._Verbose(ERROR_CODE[error])
        self._channel = index

    def SetEMAdvanced(self, gainAdvanced):
        '''
        Enable/disable access to the advanced EM gain levels

        Input:
            gainAdvanced (int) : 1 or 0 for true or false

        Output:
            None
        '''
        error = self._dll.SetEMAdvanced(gainAdvanced)
        self._Verbose(ERROR_CODE[error])

    def SetEMCCDGainMode(self, gainMode):
        '''
        Set the gain mode

        Input:
            gainMode (int) : mode

        Output:
            None
        '''
        error = self._dll.SetEMCCDGainMode(gainMode)
        self._Verbose(ERROR_CODE[error])

    def SetExposureTime(self, time_):
        '''
        Set the exposure time in seconds

        Input:
            time_ (float) : The exposure time in seconds

        Output:
            None
        '''
        error = self._dll.SetExposureTime(c_float(time_))
        self._Verbose(ERROR_CODE[error])

    def SetFrameTransferMode(self, frameTransfer):
        '''
        Enable/disable the frame transfer mode

        Input:
            frameTransfer (int) : 1 or 0 for true or false

        Output:
            None
        '''
        error = self._dll.SetFrameTransferMode(frameTransfer)
        self._Verbose(ERROR_CODE[error])

    def SetImageRotate(self, iRotate):
        '''
        Set the modus for image rotation

        Input:
            iRotate (int) : 0 for no rotation, 1 for 90 deg cw, 2 for 90 deg ccw

        Output:
            None
        '''
        error = self._dll.SetImageRotate(iRotate)
        self._Verbose(ERROR_CODE[error])

    def SetKineticCycleTime(self, time_):
        '''
        Set the Kinetic cycle time in seconds

        Input:
            time_ (float) : The cycle time in seconds

        Output:
            None
        '''
        error = self._dll.SetKineticCycleTime(c_float(time_))
        self._Verbose(ERROR_CODE[error])

    def SetNumberAccumulations(self, number):
        '''
        Set the number of scans accumulated in memory,
        for kinetic and accumulate modes

        Input:
            number (int) : The number of accumulations

        Output:
            None
        '''
        error = self._dll.SetNumberAccumulations(number)
        self._Verbose(ERROR_CODE[error])

    def SetNumberKinetics(self, numKin):
        '''
        Set the number of scans accumulated in memory for kinetic mode

        Input:
            number (int) : The number of accumulations

        Output:
            None
        '''
        error = self._dll.SetNumberKinetics(numKin)
        self._Verbose(ERROR_CODE[error])

    def SetOutputAmplifier(self, index):
        '''
        Specify which amplifier to use if EMCCD is enabled

        Input:
            index (int) : 0 for EMCCD, 1 for conventional

        Output:
            None
        '''
        error = self._dll.SetOutputAmplifier(index)
        self._Verbose(ERROR_CODE[error])
        self._outamp = index

    def SetReadMode(self, mode):
        '''
        Set the read mode of the camera

        Input:
            mode (int) : 0 Full Vertical Binning
                         1 Multi-Track
                         2 Random-track
                         3 Single-Track
                         4 Image

        Output:
            None
        '''
        error = self._dll.SetReadMode(mode)
        self._ReadMode = mode
        self._Verbose(ERROR_CODE[error])

    def SetTriggerMode(self, mode):
        '''
        Set the trigger mode

        Input:
            mode (int) : 0 Internal
                         1 External
                         2 External Start (only in Fast Kinetics mode)

        Output:
            None
        '''
        error = self._dll.SetTriggerMode(mode)
        self._Verbose(ERROR_CODE[error])

    ###### Single Parameters Get ######

    def GetAccumulationProgress(self):
        '''
        Returns the number of completed accumulations

        Input:
            None

        Output:
            (int) : The number of accumulations
        '''
        acc = c_long()
        series = c_long()
        error = self._dll.GetAcquisitionProgress(byref(acc), byref(series))
        if ERROR_CODE[error] == "DRV_SUCCESS":
            return acc.value
        else:
            return None

    def GetAcquiredData(self, imageArray):
        '''
        Returns the Acquired data

        Input:
            None

        Output:
            (array) : an array containing the acquired data
        '''
        # FIXME : Check how this works for FVB !!!
        if self._ReadMode == 0:
            dim = self._width
        elif self._ReadMode == 4:
            dim = self._width * self._height

        print("Dim is %s" % dim)
        cimageArray = c_int * dim
        cimage = cimageArray()
        error = self._dll.GetAcquiredData(pointer(cimage), dim)
        self._Verbose(ERROR_CODE[error])

        for i in range(len(cimage)):
            imageArray.append(cimage[i])

        self._imageArray = imageArray[:]
        self._Verbose(ERROR_CODE[error])
        return self._imageArray

    def GetBitDepth(self):
        '''
        Returns the bit depth of the available channels

        Input:
            None

        Output:
            (int[]) : The bit depths
        '''
        bitDepth = c_int()
        self._bitDepths = []

        for i in range(self._noADChannels):
            self._dll.GetBitDepth(i, byref(bitDepth))
            self._bitDepths.append(bitDepth.value)
        return self._bitDepths

    def GetEMGainRange(self):
        '''
        Returns the number of completed accumulations

        Input:
            None

        Output:
            int) : The number of accumulations
        '''
        low = c_int()
        high = c_int()
        error = self._dll.GetEMGainRange(byref(low), byref(high))
        self._gainRange = (low.value, high.value)
        self._Verbose(ERROR_CODE[error])
        return self._gainRange

    def GetNumberADChannels(self):
        '''
        Returns the number of AD channels

        Input:
            None

        Output:
            (int) : The number of AD channels
        '''
        noADChannels = c_int()
        error = self._dll.GetNumberADChannels(byref(noADChannels))
        self._noADChannels = noADChannels.value
        self._Verbose(ERROR_CODE[error])
        return self._noADChannels

    def GetNumberPreAmpGains(self):
        '''
        Returns the number of Pre Amp Gains

        Input:
            None

        Output:
            (int) : The number of Pre Amp Gains
        '''
        noGains = c_int()
        error = self._dll.GetNumberPreAmpGains(byref(noGains))
        self._noGains = noGains.value
        self._Verbose(ERROR_CODE[error])
        return self._noGains

    def GetSeriesProgress(self):
        '''
        Returns the number of completed kenetic scans

        Input:
            None

        Output:
            (int) : The number of completed kinetic scans
        '''
        acc = c_long()
        series = c_long()
        error = self._dll.GetAcquisitionProgress(byref(acc), byref(series))
        if ERROR_CODE[error] == "DRV_SUCCESS":
            return series.value
        else:
            return None

    def GetStatus(self):
        '''
        Returns the status of the camera

        Input:
            None

        Output:
            (string) : DRV_IDLE
                       DRV_TEMPCYCLE
                       DRV_ACQUIRING
                       DRV_TIME_NOT_MET
                       DRV_KINETIC_TIME_NOT_MET
                       DRV_ERROR_ACK
                       DRV_ACQ_BUFFER
                       DRV_SPOOLERROR
        '''
        status = c_int()
        error = self._dll.GetStatus(byref(status))
        self._status = ERROR_CODE[status.value]
        self._Verbose(ERROR_CODE[error])
        return self._status

    ###### Single Parameters Get/Set ######
    def GetEMCCDGain(self):
        '''
        Returns EMCCD Gain setting

        Input:
            None

        Output:
            (int) : EMCCD gain setting
        '''
        gain = c_int()
        error = self._dll.GetEMCCDGain(byref(gain))
        self._gain = gain.value
        self._Verbose(ERROR_CODE[error])
        return self._gain

    def SetEMCCDGain(self, gain):
        '''
        Set the EMCCD Gain setting

        Input:
            gain (int) : EMCCD setting

        Output:
            None
        '''
        error = self._dll.SetEMCCDGain(gain)
        self._Verbose(ERROR_CODE[error])

    def GetHSSpeed(self):
        '''
        Returns the available HS speeds of the selected channel

        Input:
            None

        Output:
            (float[]) : The speeds of the selected channel
        '''
        HSSpeed = c_float()
        self._HSSpeeds = []
        for i in range(self._noHSSpeeds):
            self._dll.GetHSSpeed(self._channel, self._outamp, i, byref(HSSpeed))
            self._HSSpeeds.append(HSSpeed.value)
        return self._HSSpeeds

    def SetHSSpeed(self, index):
        '''
        Set the HS speed to the mode corresponding to the index

        Input:
            index (int) : index corresponding to the Speed mode

        Output:
            None
        '''
        error = self._dll.SetHSSpeed(index)
        self._Verbose(ERROR_CODE[error])
        self._hsspeed = index

    def GetVSSpeed(self):
        '''
        Returns the available VS speeds of the selected channel

        Input:
            None

        Output:
            (float[]) : The speeds of the selected channel
        '''
        VSSpeed = c_float()
        self._VSSpeeds = []

        for i in range(self._noVSSpeeds):
            self._dll.GetVSSpeed(i, byref(VSSpeed))
            self._VSSpeeds.append(VSSpeed.value)
        return self._VSSpeeds

    def SetVSSpeed(self, index):
        '''
        Set the VS speed to the mode corresponding to the index

        Input:
            index (int) : index corresponding to the Speed mode

        Output:
            None
        '''
        error = self._dll.SetVSSpeed(index)
        self._Verbose(ERROR_CODE[error])
        self._vsspeed = index

    def GetPreAmpGain(self):
        '''
        Returns the available Pre Amp Gains

        Input:
            None

        Output:
            (float[]) : The pre amp gains
        '''
        gain = c_float()
        self._preAmpGain = []

        for i in range(self._noGains):
            self._dll.GetPreAmpGain(i, byref(gain))
            self._preAmpGain.append(gain.value)
        return self._preAmpGain

    def SetPreAmpGain(self, index):
        '''
        Set the Pre Amp Gain to the mode corresponding to the index

        Input:
            index (int) : index corresponding to the Gain mode

        Output:
            None
        '''
        error = self._dll.SetPreAmpGain(index)
        self._Verbose(ERROR_CODE[error])
        self._preampgain = index

    ###### iDus interaction Functions ######
    def ShutDown(self):  # Careful with this one!!
        '''
        Shut down the Andor
        '''
        error = self._dll.ShutDown()
        self._Verbose(ERROR_CODE[error])

    def AbortAcquisition(self):
        '''
        Abort the acquisition
        '''
        error = self._dll.AbortAcquisition()
        self._Verbose(ERROR_CODE[error])

    def StartAcquisition(self):
        '''
        Start the acquisition
        '''
        error = self._dll.StartAcquisition()
        # self._dll.WaitForAcquisition()
        self._Verbose(ERROR_CODE[error])

    def SetSingleImage(self):
        '''
        Shortcut to apply settings for a single scan full image
        '''
        self.SetReadMode(4)
        self.SetAcquisitionMode(1)
        print("Width: %d Height: %d" % (self._width, self._height))
        self.SetImage(1, 1, 1, self._width, 1, self._height)

    def SetSingleFVB(self):
        '''
        Shortcut to apply settings for a single scan FVB
        '''
        self.SetReadMode(0)
        self.SetAcquisitionMode(1)

    def GetAcquisitionTimings(self):
        '''
        Acquire all the relevant timings for acquisition,
        and store them in local memory
        '''
        exposure = c_float()
        accumulate = c_float()
        kinetic = c_float()
        error = self._dll.GetAcquisitionTimings(byref(exposure),
                                                byref(accumulate), byref(kinetic))
        self._exposure = exposure.value
        self._accumulate = accumulate.value
        self._kinetic = kinetic.value
        self._Verbose(ERROR_CODE[error])

    ###### Misc functions ######

    def SetImage(self, hbin, vbin, hstart, hend, vstart, vend):
        '''
        Specify the binning and domain of the image

        Input:
            hbin   (int) : horizontal binning
            vbin   (int) : vertical binning
            hstart (int) : horizontal starting point
            hend   (int) : horizontal end point
            vstart (int) : vertical starting point
            vend   (int) : vertical end point

        Output:
            None
        '''
        error = self._dll.SetImage(hbin, vbin, hstart, hend, vstart, vend)
        self._Verbose(ERROR_CODE[error])

    def SetShutter(self, typ, mode, closingtime, openingtime):
        '''
        Set the configuration for the shutter

        Input:
            typ         (int) : 0/1 Output TTL low/high signal to open shutter
            mode        (int) : 0/1/2 For Auto/Open/Close
            closingtime (int) : millisecs it takes to close
            openingtime (int) : millisecs it takes to open

        Output:
            None
        '''
        error = self._dll.SetShutter(typ, mode, closingtime, openingtime)
        self._Verbose(ERROR_CODE[error])

    def SetShutterEx(self, typ, mode, closingtime, openingtime, extmode):
        '''
        Set the configuration for the shutter in external mode

        Input:
            typ         (int) : 0/1 Output TTL low/high signal to open shutter
            mode        (int) : 0/1/2 For Auto/Open/Close
            closingtime (int) : millisecs it takes to close
            openingtime (int) : millisecs it takes to open
            extmode     (int) : 0/1/2 For Auto/Open/Close

        Output:
            None
        '''
        error = self._dll.SetShutterEx(typ, mode, closingtime, openingtime,
                                       extmode)
        self._Verbose(ERROR_CODE[error])

    def SetSpool(self, active, method, path, framebuffersize):
        '''
        Set Spooling. Refer to manual for detailed description
        '''
        error = self._dll.SetSpool(active, method, c_char_p(path),
                                   framebuffersize)
        self._Verbose(ERROR_CODE[error])

    def PlotButDontSave(self, read_mode=0):
        """
        Plot the image with matplotlib.
        Specifically, we sum the values of each row of data in the image, and plot that.
        Remember that the image is sort of sideways, taller than it is long.
        """
        if read_mode == 4:  # it's an image
            actual_height = len(self._imageArray) // self._width
            im = np.reshape(np.array(self._imageArray), (self._width, actual_height))

            y_vals = np.array([sum(im[i, :]) for i in range(self._width)])

            plt.plot(list(range(self._width)), y_vals, linewidth=0.3, color='red')
            plt.draw()
            plt.pause(0.05)
            input("hit ENTER to continue.")
            plt.close()

        elif read_mode == 0:  # FVB
            # x_vals = [self.get_wavelength(i) for i in range(self._width)]
            plt.plot(list(range(self._width)), self._imageArray, linewidth=0.3, color='red')
            plt.draw()
            plt.pause(0.05)
            input("hit ENTER to continue.")
            plt.close()

        else:
            pass

    def SaveAsBmp(self, path):
        '''
        Save the most recent acquired image as a bitmap

        Input:
            path (string) : Filename to save to

        Output:
            None
        '''
        im = Image.new("RGB", (self._height, self._width), "white")
        pix = im.load()

        print(self._imageArray)
        for i in range(len(self._imageArray)):
            (row, col) = divmod(i, self._width)
            picvalue = int(round(self._imageArray[i] * 255.0 / 65535))
            print(self.imageArray[i])
            print(type(self.imageArray[i]))
            pix[row, col] = (picvalue, picvalue, picvalue)

        im.save(path, "BMP")
        print("savedj?")

    def SaveAsTxt(self, path):
        '''
        Save the most recent acquired image as txt

        Input:
            path (string) : Filename to save to

        Output:
            None
        '''
        with open(path, 'w') as filename:
            print("saving as", path)

            for line in self._imageArray:
                filename.write("%g\n" % line)

    def SaveAsBmpNormalised(self, path):
        '''
        Save the most recent acquired image as a bitmap,
        but maximize contrast

        Input:
            path (string) : Filename to save to

        Output:
            None
        '''
        im = Image.new("RGB", (self._height, self._width), "white")
        pix = im.load()
        maxIntensity = max(self._imageArray)
        minIntensity = min(self._imageArray)
        print(maxIntensity, minIntensity)
        for i in range(len(self._imageArray)):
            (row, col) = divmod(i, self._width)
            picvalue = int(round((self._imageArray[i] - minIntensity) * 255.0 /
                                 (maxIntensity - minIntensity)))
            pix[row, col] = (picvalue, picvalue, picvalue)
        im.save(path, "BMP")

    def SaveAsFITS(self, filename, type_):
        '''
        Save the most recent acquired image as FITS

        Input:
            path (string) : Filename to save to

        Output:
            None
        '''
        error = self._dll.SaveAsFITS(filename, type_)
        self._Verbose(ERROR_CODE[error])

    ########### Automation functions #################

    def CoolDown(self, Tset=-25):
        '''
        Cool down the camera for a measurement
        '''
        curframe = inspect.currentframe()
        calframe = inspect.getouterframes(curframe, 2)
        print('cooldown caller name:', calframe[1][3])
        self.SetCoolerMode(0)  # return to ambient temperature on ShutDown

        self.SetTemperature(Tset)
        self.CoolerON()

        print("cooling down...")
        while (self.GetTemperature() is not 'DRV_TEMP_STABILIZED') and \
                (self.emergency_stop_cooling != True):
            if debugging == True:
                print("emergency?", self.emergency_stop_cooling)
            time.sleep(1)

    def WarmUp(self):
        '''
        Warm up after a measurement
        '''
        target_temperature = -20  # degrees C; don't shut camera down before

        if self._cooler_on:
            print("emergency cooler off")
            self.emergency_stop_cooling = True

        self.CoolerOFF()

        if self._temperature < target_temperature:
            print("warming from WarmUp")
            while self._temperature < target_temperature:
                self.GetTemperature()
                time.sleep(1)
            print("camera ready to shut down.")
        else:
            print("camera ready to shut down.")
            pass

    def Demo_ImagePrepare(self):
        '''
        Prepare the camera for a demo image measurement
        '''
        PreAmpGain = 0
        self.SetSingleImage()
        self.SetTriggerMode(0)
        self.SetShutter(1, 1, 0, 0)
        self.SetPreAmpGain(PreAmpGain)
        self.SetExposureTime(self._exposure)

    def ImageCapture(self):
        '''
        Perform the demo image measurement. The saving is done by the
        main view.
        '''
        i = 0
        while i < 1:  # previously i < 4
            i += 1
            print(self.GetTemperature())
            print(self._temperature)
            print("Ready for Acquisition")
            self._currently_taking_data = True
            self.announce_update_to_model()  # to announce exposure
            # so that the LED lights up
            self.StartAcquisition()

            # Check for status
            i = 0
            while self.GetStatus() is not 'DRV_IDLE':
                print("About {} s remaining.".format(
                    self.set_exposure_time - i))
                i += 0.5
                time.sleep(0.5)

            self._currently_taking_data = False
            self.announce_update_to_model()
            print("Done getting image.")
            data = []
            self.GetAcquiredData(data)
            print("data acquired")
            # self.true_path = self.set_full_data_path + ".bmp"
            # self.SaveAsBmp(self.true_path)
            # ^ problem is here
            # self.true_path = self.set_full_data_path + ".txt" # ugh
            # self.SaveAsTxt(self.true_path)

    def Demo_FVBPrepare(self):
        '''
        Prepare the camera for a demo image measurement
        '''
        PreAmpGain = 0
        self.SetSingleFVB()
        self.SetTriggerMode(0)
        self.SetShutter(1, 1, 0, 0)
        self.SetPreAmpGain(PreAmpGain)
        self.SetExposureTime(self._exposure)

    def FVBCapture(self):
        '''
        Perform the demo image measurement
        '''
        i = 0
        while i < 1:
            print("-" * 20)
            print(self._temperature)
            i += 1
            print(self.GetTemperature())
            print(self._temperature)
            print("Ready for Acquisition")
            self.StartAcquisition()

            # Check for status
            while self.GetStatus() is not 'DRV_IDLE':
                print("Data not yet acquired, waiting 0.5s")
                time.sleep(0.5)

            data = []
            self.GetAcquiredData(data)  # puts data in _imageArray
            # self.true_path =\
            #    self.set_full_data_path + "_{0}.txt".format(i)
            self.true_path = self.set_full_data_path + ".txt"
            self.SaveAsTxt(self.true_path)
            # self.SaveAsBmp(self.true_path)
            # self.PlotButDontSave(0)


# class DummyCamera(object):
#    """
#    For testing.
#    """
#    def __init__(self):
#        print "Turning on."
#        self.set_trigger_type = 0 # 0 = internal; 1 = external
#        self.set_temperature = 0 # degrees celcius
#        self.set_exposure_time = 0 # seconds. change to none
#                                   # (for the2 lines above too)
#        self.set_data_name = ""
#        self.set_full_data_path = ""
#        self.true_path = "" # ugly
#        self.set_aquisition_mode = ""
#        self._should_i_plot = False
#        self.should_i_start = True
#        self._view_funcs = []
#
#        self._width        = 1
#        self._height       = 1
#        self._temperature  = 0
#        self._temp_min     = -20 # minumum temperature camera can be cooled.
#        self._temp_max     = 20 # (see GetTemperatureRange)
#        self._set_T        = 0 # change to None
#        self._gain         = 1
#        self._gainRange    = 1
#        self._status       = "Fine"
#        self._verbosity    = True
#        self._preampgain   = 1
#        self._channel      = 1
#        self._outamp       = 1
#        self._hsspeed      = 1
#        self._vsspeed      = 1
#        self._serial       = 1
#        self._exposure     = 0
#        self._accumulate   = 1
#        self._kinetic      = 1
#        self._bitDepths    = []
#        self._preAmpGain   = []
#        self._VSSpeeds     = []
#        self._noGains      = None
#        self._imageArray   = []
#        self._noVSSpeeds   = None
#        self._HSSpeeds     = []
#        self._noADChannels = None
#        self._noHSSpeeds   = None
#        self._ReadMode     = 0
#        self._cooler_on    = False
#
#    def __del__(self):
#        print "Shutting down."
#
#    def start_data_acquisition(self):
#        self.implement_parameters()
#
#    def implement_parameters(self, str=''):
#        """
#        Main work loop.
#        """
#        print str, "!!!"
#        print "IMPLEMENTING"
#        i = 0
#        while i < 10:
#            print i
#            i += 1
#            time.sleep(1)
#
#        print "Plotting."
#
#    def announce_update_to_model(self):
#        print "announcing update..."
#        for func in self._view_funcs:
#            func()

# List of error codes
ERROR_CODE = {
    20001: "DRV_ERROR_CODES",
    20002: "DRV_SUCCESS",
    20003: "DRV_VXNOTINSTALLED",
    20006: "DRV_ERROR_FILELOAD",
    20007: "DRV_ERROR_VXD_INIT",
    20010: "DRV_ERROR_PAGELOCK",
    20011: "DRV_ERROR_PAGE_UNLOCK",
    20013: "DRV_ERROR_ACK",
    20024: "DRV_NO_NEW_DATA",
    20026: "DRV_SPOOLERROR",
    20034: "DRV_TEMP_OFF",
    20035: "DRV_TEMP_NOT_STABILIZED",
    20036: "DRV_TEMP_STABILIZED",
    20037: "DRV_TEMP_NOT_REACHED",
    20038: "DRV_TEMP_OUT_RANGE",
    20039: "DRV_TEMP_NOT_SUPPORTED",
    20040: "DRV_TEMP_DRIFT",
    20050: "DRV_COF_NOTLOADED",
    20053: "DRV_FLEXERROR",
    20066: "DRV_P1INVALID",
    20067: "DRV_P2INVALID",
    20068: "DRV_P3INVALID",
    20069: "DRV_P4INVALID",
    20070: "DRV_INIERROR",
    20071: "DRV_COERROR",
    20072: "DRV_ACQUIRING",
    20073: "DRV_IDLE",
    20074: "DRV_TEMPCYCLE",
    20075: "DRV_NOT_INITIALIZED",
    20076: "DRV_P5INVALID",
    20077: "DRV_P6INVALID",
    20083: "P7_INVALID",
    20089: "DRV_USBERROR",
    20091: "DRV_NOT_SUPPORTED",
    20099: "DRV_BINNING_ERROR",
    20990: "DRV_NOCAMERA",
    20991: "DRV_NOT_SUPPORTED",
    20992: "DRV_NOT_AVAILABLE"
}
