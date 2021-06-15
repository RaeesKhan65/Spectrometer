import os.path
import time
import sys
from PyQt5 import QtCore, QtWidgets, uic
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import argrelextrema

plot = False


def calibrate(spectrum_file, calibration_file, which_peak=703.24, location_of_peak=560):
    """
    Finds the maxima of a spectrum for calibration purposes.
    The peak_locations variable is the list of peaks given by
    the calibration file.

    location_of_peak is the pixel location of the peak specified by which_peak
    which_peak is the peak in nm that we can identify.

    Returns the function get_wavelength which converts pixel
    to wavelength.
    """
    data = []
    peak_locations = []

    with open(spectrum_file) as data_file:
        for line in data_file:
            if not line.startswith('#'):
                data.append(float(line))

    im = np.reshape(np.array(data), (127, 1024))
    x_vals = list(range(1024))
    y_vals = np.array([np.mean(im[10:, i]) for i in range(1024)])
    y_vals = y_vals[::-1]

    maxima_locations = get_maxima_locations(y_vals)

    with open(calibration_file) as cal_file:
        for line in cal_file:
            if not line.startswith('#'):
                peak_locations.append(float(line))

    data = np.array(data)
    peak_locations = np.array(peak_locations)

    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=False)

    # need user to find pixel location of the 703 nm peak.
    x0 = location_of_peak
    user_peak_index = list(maxima_locations).index(location_of_peak)
    calibration_peak_index = list(peak_locations).index(which_peak)

    # then we need to clip the calibration list so that it's the same
    # length as the data. here the data list is longer.
    # remember peak_locations = calibration peaks
    left_length_cal = len(peak_locations[:calibration_peak_index])
    right_length_cal = len(peak_locations[calibration_peak_index:])

    left_length_user = len(maxima_locations[:user_peak_index])
    right_length_user = len(maxima_locations[user_peak_index:])

    llc = left_length_cal
    rlc = right_length_cal
    llu = left_length_user
    rlu = right_length_user  # to make code easier to read

    print(np.abs(llu - llc), np.abs(rlu - rlc))
    print("---")
    print(llc, rlc)
    print(llu, rlu)
    print("---")

    if len(maxima_locations) < len(peak_locations):  # usually is
        print("good")

    else:
        print("it isn't")
        print(len(maxima_locations))
        print(len(peak_locations))

    if llc >= llu:
        peak_locations = peak_locations[(llc - llu):]
    elif llc < llu:
        maxima_locations = maxima_locations[(llu - llc):]

    if rlc >= rlu:
        peak_locations = peak_locations[:(-(rlc - rlu) or None)]
    elif rlc < rlu:
        maxima_locations = maxima_locations[:-(rlu - rlc)]
    # todo: include limit cases with small lists

    print("---")
    print(len(maxima_locations))
    print("---")
    print(len(peak_locations))

    # now we scale the user data
    s = np.polyfit(maxima_locations, peak_locations, 1)
    # ^should include R

    mm, bb = s  # bb because there's some other variable named 'b'
    print("m:", mm)
    print("b:", bb)

    def get_wavelength(x):
        """Converts pixel to wavelength."""
        return mm * x + bb

    final_peaks = [
        get_wavelength(i) for i in maxima_locations]

    final_x = [
        get_wavelength(i) for i in range(1024)]

    final_diffs = sum([
        (a - b) ** 2 for (a, b) in zip(final_peaks, peak_locations)])

    print("Least squares:", final_diffs)

    if plot == True:
        ax1.plot(maxima_locations, peak_locations, 'r.')
        ax1.plot(maxima_locations, final_peaks)

        ax2.plot(final_x, y_vals, 'k-')

        for point in peak_locations:
            ax2.axvline(x=point, linewidth=0.5,
                        color='green')

        new_max = get_maxima_locations(y_vals)
        ax2.scatter([final_x[i] for i in new_max],
                    [y_vals[i] for i in new_max])

        ax1.set_xlabel("Pixel")
        ax2.set_xlabel("Wavelength (nm)")

        plt.draw()
        plt.show()
    else:
        print("calibration done.")

    return get_wavelength, [
        maxima_locations, peak_locations, final_peaks,
        final_x, y_vals, final_diffs, mm, bb]


def get_maxima_locations(y_vals):
    """
    Return the indices of the maxima for the input y_vals.
    """
    maxima_locations = list(
        argrelextrema(y_vals, np.greater, order=3))[0]
    # ^ gives indices of relative maxima

    std = np.std(y_vals)
    m = 4  # for determining if a maximum is a true peak
    print(std, m * std)

    maxima_locations = [
        loc for loc in maxima_locations if not \
            (y_vals[loc] < (min(y_vals) + std * 0.2))]
    # ^ make sure they aren't tiny false peaks

    return maxima_locations


if __name__ == '__main__':
    plot = True
    if len(sys.argv) != 3:
        print("usage: python calibrate.py path_to_file " \
              "path_to_calibration_file")
    else:
        calibrate(sys.argv[1], sys.argv[2])
