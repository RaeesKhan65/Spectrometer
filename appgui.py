# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'modified_spectrometer.ui'
#
# Created by: PyQt5 UI code generator 5.15.4
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Spectrometer(object):
    def setupUi(self, Spectrometer):
        Spectrometer.setObjectName("Spectrometer")
        Spectrometer.resize(1121, 900)
        Spectrometer.setFocusPolicy(QtCore.Qt.StrongFocus)
        Spectrometer.setStyleSheet("")
        self.centralwidget = QtWidgets.QWidget(Spectrometer)
        self.centralwidget.setObjectName("centralwidget")
        self.tab_selector = QtWidgets.QTabWidget(self.centralwidget)
        self.tab_selector.setGeometry(QtCore.QRect(0, 0, 1121, 891))
        self.tab_selector.setObjectName("tab_selector")
        self.take_data_tab = QtWidgets.QWidget()
        self.take_data_tab.setObjectName("take_data_tab")
        self.error_window = QtWidgets.QLabel(self.take_data_tab)
        self.error_window.setGeometry(QtCore.QRect(10, 810, 911, 35))
        self.error_window.setStyleSheet("font: 10pt \"Fixedsys\";")
        self.error_window.setObjectName("error_window")
        self.camera_group = QtWidgets.QGroupBox(self.take_data_tab)
        self.camera_group.setGeometry(QtCore.QRect(10, 10, 261, 121))
        self.camera_group.setObjectName("camera_group")
        self.layoutWidget_2 = QtWidgets.QWidget(self.camera_group)
        self.layoutWidget_2.setGeometry(QtCore.QRect(10, 20, 241, 91))
        self.layoutWidget_2.setObjectName("layoutWidget_2")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.layoutWidget_2)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.label_4 = QtWidgets.QLabel(self.layoutWidget_2)
        self.label_4.setObjectName("label_4")
        self.verticalLayout.addWidget(self.label_4)
        self.label_3 = QtWidgets.QLabel(self.layoutWidget_2)
        self.label_3.setObjectName("label_3")
        self.verticalLayout.addWidget(self.label_3)
        self.label_2 = QtWidgets.QLabel(self.layoutWidget_2)
        self.label_2.setObjectName("label_2")
        self.verticalLayout.addWidget(self.label_2)
        self.horizontalLayout.addLayout(self.verticalLayout)
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.temperature_box = QtWidgets.QLineEdit(self.layoutWidget_2)
        self.temperature_box.setObjectName("temperature_box")
        self.verticalLayout_2.addWidget(self.temperature_box)
        self.exposure_time_box = QtWidgets.QLineEdit(self.layoutWidget_2)
        self.exposure_time_box.setObjectName("exposure_time_box")
        self.verticalLayout_2.addWidget(self.exposure_time_box)
        self.trigger_selector = QtWidgets.QComboBox(self.layoutWidget_2)
        self.trigger_selector.setObjectName("trigger_selector")
        self.trigger_selector.addItem("")
        self.trigger_selector.addItem("")
        self.verticalLayout_2.addWidget(self.trigger_selector)
        self.horizontalLayout.addLayout(self.verticalLayout_2)
        self.shut_down_button = QtWidgets.QPushButton(self.take_data_tab)
        self.shut_down_button.setGeometry(QtCore.QRect(950, 170, 151, 61))
        self.shut_down_button.setStyleSheet("background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #ff0000, stop: 1 #cf161e);")
        self.shut_down_button.setObjectName("shut_down_button")
        self.graph_box = MplWidget(self.take_data_tab)
        self.graph_box.setGeometry(QtCore.QRect(10, 140, 931, 671))
        self.graph_box.setObjectName("graph_box")
        self.data_settings = QtWidgets.QGroupBox(self.take_data_tab)
        self.data_settings.setGeometry(QtCore.QRect(280, 10, 301, 121))
        self.data_settings.setObjectName("data_settings")
        self.select_path = QtWidgets.QPushButton(self.data_settings)
        self.select_path.setGeometry(QtCore.QRect(10, 90, 75, 23))
        self.select_path.setObjectName("select_path")
        self.layoutWidget_3 = QtWidgets.QWidget(self.data_settings)
        self.layoutWidget_3.setGeometry(QtCore.QRect(10, 20, 281, 61))
        self.layoutWidget_3.setObjectName("layoutWidget_3")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.layoutWidget_3)
        self.horizontalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout()
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.label_6 = QtWidgets.QLabel(self.layoutWidget_3)
        self.label_6.setObjectName("label_6")
        self.verticalLayout_4.addWidget(self.label_6)
        self.label_5 = QtWidgets.QLabel(self.layoutWidget_3)
        self.label_5.setObjectName("label_5")
        self.verticalLayout_4.addWidget(self.label_5)
        self.horizontalLayout_2.addLayout(self.verticalLayout_4)
        self.verticalLayout_3 = QtWidgets.QVBoxLayout()
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.data_name_box = QtWidgets.QLineEdit(self.layoutWidget_3)
        self.data_name_box.setMaximumSize(QtCore.QSize(181, 16777215))
        self.data_name_box.setObjectName("data_name_box")
        self.verticalLayout_3.addWidget(self.data_name_box)
        self.path_name = QtWidgets.QLineEdit(self.layoutWidget_3)
        self.path_name.setObjectName("path_name")
        self.verticalLayout_3.addWidget(self.path_name)
        self.horizontalLayout_2.addLayout(self.verticalLayout_3)
        self.layoutWidget_3.raise_()
        self.select_path.raise_()
        self.start_button = QtWidgets.QPushButton(self.take_data_tab)
        self.start_button.setGeometry(QtCore.QRect(950, 10, 151, 71))
        self.start_button.setStyleSheet("")
        self.start_button.setObjectName("start_button")
        self.stop_cooler_button = QtWidgets.QPushButton(self.take_data_tab)
        self.stop_cooler_button.setGeometry(QtCore.QRect(950, 90, 151, 71))
        self.stop_cooler_button.setStyleSheet("")
        self.stop_cooler_button.setObjectName("stop_cooler_button")
        self.layoutWidget = QtWidgets.QWidget(self.take_data_tab)
        self.layoutWidget.setGeometry(QtCore.QRect(951, 240, 151, 111))
        self.layoutWidget.setObjectName("layoutWidget")
        self.verticalLayout_5 = QtWidgets.QVBoxLayout(self.layoutWidget)
        self.verticalLayout_5.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.label_7 = QtWidgets.QLabel(self.layoutWidget)
        self.label_7.setObjectName("label_7")
        self.horizontalLayout_3.addWidget(self.label_7)
        spacerItem = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_3.addItem(spacerItem)
        self.temperature_display = QtWidgets.QLCDNumber(self.layoutWidget)
        self.temperature_display.setStyleSheet("")
        self.temperature_display.setMode(QtWidgets.QLCDNumber.Dec)
        self.temperature_display.setProperty("value", 0.0)
        self.temperature_display.setProperty("intValue", 0)
        self.temperature_display.setObjectName("temperature_display")
        self.horizontalLayout_3.addWidget(self.temperature_display)
        self.verticalLayout_5.addLayout(self.horizontalLayout_3)
        self.cooler_status = QtWidgets.QLabel(self.layoutWidget)
        self.cooler_status.setObjectName("cooler_status")
        self.verticalLayout_5.addWidget(self.cooler_status)
        self.continue_taking_data_box = QtWidgets.QCheckBox(self.layoutWidget)
        self.continue_taking_data_box.setChecked(True)
        self.continue_taking_data_box.setObjectName("continue_taking_data_box")
        self.verticalLayout_5.addWidget(self.continue_taking_data_box)
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.label = QtWidgets.QLabel(self.layoutWidget)
        self.label.setMinimumSize(QtCore.QSize(91, 0))
        self.label.setObjectName("label")
        self.horizontalLayout_4.addWidget(self.label)
        spacerItem1 = QtWidgets.QSpacerItem(48, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_4.addItem(spacerItem1)
        self.exposure_led = QLed(self.layoutWidget)
        self.exposure_led.setObjectName("exposure_led")
        self.horizontalLayout_4.addWidget(self.exposure_led)
        self.verticalLayout_5.addLayout(self.horizontalLayout_4)
        self.motor_settings = QtWidgets.QGroupBox(self.take_data_tab)
        self.motor_settings.setGeometry(QtCore.QRect(950, 360, 151, 111))
        self.motor_settings.setMinimumSize(QtCore.QSize(151, 0))
        self.motor_settings.setObjectName("motor_settings")
        self.layoutWidget1 = QtWidgets.QWidget(self.motor_settings)
        self.layoutWidget1.setGeometry(QtCore.QRect(10, 20, 131, 82))
        self.layoutWidget1.setObjectName("layoutWidget1")
        self.verticalLayout_6 = QtWidgets.QVBoxLayout(self.layoutWidget1)
        self.verticalLayout_6.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_6.setObjectName("verticalLayout_6")
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.step_size_label = QtWidgets.QLabel(self.layoutWidget1)
        self.step_size_label.setObjectName("step_size_label")
        self.horizontalLayout_5.addWidget(self.step_size_label)
        self.step_size_box = QtWidgets.QLineEdit(self.layoutWidget1)
        self.step_size_box.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.step_size_box.setObjectName("step_size_box")
        self.horizontalLayout_5.addWidget(self.step_size_box)
        self.verticalLayout_6.addLayout(self.horizontalLayout_5)
        self.move_motor_up = QtWidgets.QPushButton(self.layoutWidget1)
        self.move_motor_up.setObjectName("move_motor_up")
        self.verticalLayout_6.addWidget(self.move_motor_up)
        self.move_motor_down = QtWidgets.QPushButton(self.layoutWidget1)
        self.move_motor_down.setObjectName("move_motor_down")
        self.verticalLayout_6.addWidget(self.move_motor_down)
        self.shut_down_button.raise_()
        self.graph_box.raise_()
        self.data_settings.raise_()
        self.start_button.raise_()
        self.stop_cooler_button.raise_()
        self.layoutWidget.raise_()
        self.error_window.raise_()
        self.camera_group.raise_()
        self.motor_settings.raise_()
        self.tab_selector.addTab(self.take_data_tab, "")
        self.calibration_tab = QtWidgets.QWidget()
        self.calibration_tab.setObjectName("calibration_tab")
        self.calibration_graph = MplWidget(self.calibration_tab)
        self.calibration_graph.setGeometry(QtCore.QRect(500, 170, 601, 391))
        self.calibration_graph.setObjectName("calibration_graph")
        self.neon_spectrum = QtWidgets.QLabel(self.calibration_tab)
        self.neon_spectrum.setGeometry(QtCore.QRect(30, 230, 471, 301))
        self.neon_spectrum.setObjectName("neon_spectrum")
        self.calibration_data_box = MplWidget(self.calibration_tab)
        self.calibration_data_box.setGeometry(QtCore.QRect(10, 600, 1101, 241))
        self.calibration_data_box.setObjectName("calibration_data_box")
        self.groupBox = QtWidgets.QGroupBox(self.calibration_tab)
        self.groupBox.setGeometry(QtCore.QRect(10, 10, 481, 201))
        self.groupBox.setObjectName("groupBox")
        self.layoutWidget2 = QtWidgets.QWidget(self.groupBox)
        self.layoutWidget2.setGeometry(QtCore.QRect(10, 154, 209, 31))
        self.layoutWidget2.setObjectName("layoutWidget2")
        self.horizontalLayout_8 = QtWidgets.QHBoxLayout(self.layoutWidget2)
        self.horizontalLayout_8.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_8.setObjectName("horizontalLayout_8")
        self.save_cal_button = QtWidgets.QPushButton(self.layoutWidget2)
        self.save_cal_button.setObjectName("save_cal_button")
        self.horizontalLayout_8.addWidget(self.save_cal_button)
        self.load_cal_button = QtWidgets.QPushButton(self.layoutWidget2)
        self.load_cal_button.setObjectName("load_cal_button")
        self.horizontalLayout_8.addWidget(self.load_cal_button)
        self.layoutWidget3 = QtWidgets.QWidget(self.groupBox)
        self.layoutWidget3.setGeometry(QtCore.QRect(10, 26, 466, 121))
        self.layoutWidget3.setObjectName("layoutWidget3")
        self.verticalLayout_7 = QtWidgets.QVBoxLayout(self.layoutWidget3)
        self.verticalLayout_7.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_7.setObjectName("verticalLayout_7")
        self.horizontalLayout_15 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_15.setObjectName("horizontalLayout_15")
        self.label_9 = QtWidgets.QLabel(self.layoutWidget3)
        self.label_9.setObjectName("label_9")
        self.horizontalLayout_15.addWidget(self.label_9)
        self.calibration_spectrum_path = QtWidgets.QLineEdit(self.layoutWidget3)
        self.calibration_spectrum_path.setObjectName("calibration_spectrum_path")
        self.horizontalLayout_15.addWidget(self.calibration_spectrum_path)
        self.load_calibration_spectrum_button = QtWidgets.QPushButton(self.layoutWidget3)
        self.load_calibration_spectrum_button.setObjectName("load_calibration_spectrum_button")
        self.horizontalLayout_15.addWidget(self.load_calibration_spectrum_button)
        self.spectrum_reset = QtWidgets.QPushButton(self.layoutWidget3)
        self.spectrum_reset.setObjectName("spectrum_reset")
        self.horizontalLayout_15.addWidget(self.spectrum_reset)
        self.verticalLayout_7.addLayout(self.horizontalLayout_15)
        self.horizontalLayout_14 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_14.setObjectName("horizontalLayout_14")
        self.label_11 = QtWidgets.QLabel(self.layoutWidget3)
        self.label_11.setMinimumSize(QtCore.QSize(161, 0))
        self.label_11.setObjectName("label_11")
        self.horizontalLayout_14.addWidget(self.label_11)
        self.peak_input_box = QtWidgets.QLineEdit(self.layoutWidget3)
        self.peak_input_box.setMaxLength(32769)
        self.peak_input_box.setObjectName("peak_input_box")
        self.horizontalLayout_14.addWidget(self.peak_input_box)
        self.verticalLayout_7.addLayout(self.horizontalLayout_14)
        self.horizontalLayout_13 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_13.setObjectName("horizontalLayout_13")
        self.label_8 = QtWidgets.QLabel(self.layoutWidget3)
        self.label_8.setMinimumSize(QtCore.QSize(161, 0))
        self.label_8.setObjectName("label_8")
        self.horizontalLayout_13.addWidget(self.label_8)
        self.pixel_input_box = QtWidgets.QLineEdit(self.layoutWidget3)
        self.pixel_input_box.setText("")
        self.pixel_input_box.setMaxLength(32769)
        self.pixel_input_box.setObjectName("pixel_input_box")
        self.horizontalLayout_13.addWidget(self.pixel_input_box)
        self.calibration_button = QtWidgets.QPushButton(self.layoutWidget3)
        self.calibration_button.setObjectName("calibration_button")
        self.horizontalLayout_13.addWidget(self.calibration_button)
        self.calibration_reset = QtWidgets.QPushButton(self.layoutWidget3)
        self.calibration_reset.setObjectName("calibration_reset")
        self.horizontalLayout_13.addWidget(self.calibration_reset)
        self.verticalLayout_7.addLayout(self.horizontalLayout_13)
        self.horizontalLayout_12 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_12.setObjectName("horizontalLayout_12")
        self.label_10 = QtWidgets.QLabel(self.layoutWidget3)
        self.label_10.setObjectName("label_10")
        self.horizontalLayout_12.addWidget(self.label_10)
        self.background_path = QtWidgets.QLineEdit(self.layoutWidget3)
        self.background_path.setObjectName("background_path")
        self.horizontalLayout_12.addWidget(self.background_path)
        self.select_background_button = QtWidgets.QPushButton(self.layoutWidget3)
        self.select_background_button.setObjectName("select_background_button")
        self.horizontalLayout_12.addWidget(self.select_background_button)
        self.reset_background_button = QtWidgets.QPushButton(self.layoutWidget3)
        self.reset_background_button.setObjectName("reset_background_button")
        self.horizontalLayout_12.addWidget(self.reset_background_button)
        self.verticalLayout_7.addLayout(self.horizontalLayout_12)
        self.layoutWidget.raise_()
        self.layoutWidget.raise_()
        self.label_15 = QtWidgets.QLabel(self.calibration_tab)
        self.label_15.setGeometry(QtCore.QRect(510, 20, 421, 71))
        self.label_15.setText("")
        self.label_15.setObjectName("label_15")
        self.label_16 = QtWidgets.QLabel(self.calibration_tab)
        self.label_16.setGeometry(QtCore.QRect(500, 10, 441, 131))
        self.label_16.setObjectName("label_16")
        self.tab_selector.addTab(self.calibration_tab, "")
        self.display_data_tab = QtWidgets.QWidget()
        self.display_data_tab.setObjectName("display_data_tab")
        self.display_box = MplWidget(self.display_data_tab)
        self.display_box.setGeometry(QtCore.QRect(10, 100, 1101, 721))
        self.display_box.setObjectName("display_box")
        self.maximize_button = QtWidgets.QPushButton(self.display_data_tab)
        self.maximize_button.setGeometry(QtCore.QRect(1030, 830, 75, 23))
        self.maximize_button.setObjectName("maximize_button")
        self.layoutWidget4 = QtWidgets.QWidget(self.display_data_tab)
        self.layoutWidget4.setGeometry(QtCore.QRect(10, 10, 1011, 51))
        self.layoutWidget4.setObjectName("layoutWidget4")
        self.horizontalLayout_10 = QtWidgets.QHBoxLayout(self.layoutWidget4)
        self.horizontalLayout_10.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_10.setObjectName("horizontalLayout_10")
        self.horizontalLayout_9 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_9.setObjectName("horizontalLayout_9")
        self.horizontalLayout_7 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_7.setObjectName("horizontalLayout_7")
        self.display_path = QtWidgets.QLineEdit(self.layoutWidget4)
        self.display_path.setObjectName("display_path")
        self.horizontalLayout_7.addWidget(self.display_path)
        self.select_display_data_button = QtWidgets.QPushButton(self.layoutWidget4)
        self.select_display_data_button.setObjectName("select_display_data_button")
        self.horizontalLayout_7.addWidget(self.select_display_data_button)
        self.add_display_data_button = QtWidgets.QPushButton(self.layoutWidget4)
        self.add_display_data_button.setObjectName("add_display_data_button")
        self.horizontalLayout_7.addWidget(self.add_display_data_button)
        self.reset_display_button = QtWidgets.QPushButton(self.layoutWidget4)
        self.reset_display_button.setObjectName("reset_display_button")
        self.horizontalLayout_7.addWidget(self.reset_display_button)
        self.horizontalLayout_9.addLayout(self.horizontalLayout_7)
        self.save_data_button = QtWidgets.QPushButton(self.layoutWidget4)
        self.save_data_button.setStyleSheet("background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #ff0000, stop: 1 #cf161e);")
        self.save_data_button.setObjectName("save_data_button")
        self.horizontalLayout_9.addWidget(self.save_data_button)
        self.horizontalLayout_10.addLayout(self.horizontalLayout_9)
        self.horizontalLayout_6 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        self.cal_path = QtWidgets.QLineEdit(self.layoutWidget4)
        self.cal_path.setObjectName("cal_path")
        self.horizontalLayout_6.addWidget(self.cal_path)
        self.select_cal_file_button = QtWidgets.QPushButton(self.layoutWidget4)
        self.select_cal_file_button.setObjectName("select_cal_file_button")
        self.horizontalLayout_6.addWidget(self.select_cal_file_button)
        self.reset_display_cal_button = QtWidgets.QPushButton(self.layoutWidget4)
        self.reset_display_cal_button.setObjectName("reset_display_cal_button")
        self.horizontalLayout_6.addWidget(self.reset_display_cal_button)
        self.horizontalLayout_10.addLayout(self.horizontalLayout_6)
        self.layoutWidget5 = QtWidgets.QWidget(self.display_data_tab)
        self.layoutWidget5.setGeometry(QtCore.QRect(10, 60, 431, 41))
        self.layoutWidget5.setObjectName("layoutWidget5")
        self.horizontalLayout_11 = QtWidgets.QHBoxLayout(self.layoutWidget5)
        self.horizontalLayout_11.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_11.setObjectName("horizontalLayout_11")
        self.label_17 = QtWidgets.QLabel(self.layoutWidget5)
        self.label_17.setObjectName("label_17")
        self.horizontalLayout_11.addWidget(self.label_17)
        self.y_min_box = QtWidgets.QLineEdit(self.layoutWidget5)
        self.y_min_box.setObjectName("y_min_box")
        self.horizontalLayout_11.addWidget(self.y_min_box)
        self.label_19 = QtWidgets.QLabel(self.layoutWidget5)
        self.label_19.setObjectName("label_19")
        self.horizontalLayout_11.addWidget(self.label_19)
        self.y_max_box = QtWidgets.QLineEdit(self.layoutWidget5)
        self.y_max_box.setObjectName("y_max_box")
        self.horizontalLayout_11.addWidget(self.y_max_box)
        self.layoutWidget.raise_()
        self.display_box.raise_()
        self.layoutWidget.raise_()
        self.maximize_button.raise_()
        self.tab_selector.addTab(self.display_data_tab, "")
        Spectrometer.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(Spectrometer)
        self.statusbar.setObjectName("statusbar")
        Spectrometer.setStatusBar(self.statusbar)

        self.retranslateUi(Spectrometer)
        self.tab_selector.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(Spectrometer)

    def retranslateUi(self, Spectrometer):
        _translate = QtCore.QCoreApplication.translate
        Spectrometer.setWindowTitle(_translate("Spectrometer", "MainWindow"))
        self.error_window.setText(_translate("Spectrometer", "Camera output"))
        self.camera_group.setTitle(_translate("Spectrometer", "Camera settings"))
        self.label_4.setText(_translate("Spectrometer", "Temperature (°C)"))
        self.label_3.setText(_translate("Spectrometer", "Exposure time (s)"))
        self.label_2.setText(_translate("Spectrometer", "Trigger"))
        self.temperature_box.setText(_translate("Spectrometer", "-75"))
        self.exposure_time_box.setText(_translate("Spectrometer", "0.01"))
        self.trigger_selector.setItemText(0, _translate("Spectrometer", "Internal"))
        self.trigger_selector.setItemText(1, _translate("Spectrometer", "External"))
        self.shut_down_button.setText(_translate("Spectrometer", "Shut Down"))
        self.data_settings.setTitle(_translate("Spectrometer", "Data settings"))
        self.select_path.setText(_translate("Spectrometer", "Change path"))
        self.label_6.setText(_translate("Spectrometer", "Data Name"))
        self.label_5.setText(_translate("Spectrometer", "Saved data path"))
        self.data_name_box.setText(_translate("Spectrometer", "throwaway"))
        self.path_name.setText(_translate("Spectrometer", "H:\\Entanglement Experiments\\Data\\Spectrum"))
        self.start_button.setText(_translate("Spectrometer", "Get data"))
        self.stop_cooler_button.setText(_translate("Spectrometer", "Turn off cooler\n"
"without shutting down"))
        self.label_7.setText(_translate("Spectrometer", "Current\n"
"temperature"))
        self.cooler_status.setText(_translate("Spectrometer", "Cooler is off."))
        self.continue_taking_data_box.setText(_translate("Spectrometer", "Keep cooler on"))
        self.label.setText(_translate("Spectrometer", "Taking data when\n"
"button is lit."))
        self.motor_settings.setTitle(_translate("Spectrometer", "Motor settings"))
        self.step_size_label.setText(_translate("Spectrometer", "Step Size"))
        self.step_size_box.setText(_translate("Spectrometer", "00001"))
        self.move_motor_up.setText(_translate("Spectrometer", "Move up"))
        self.move_motor_down.setText(_translate("Spectrometer", "Move down"))
        self.tab_selector.setTabText(self.tab_selector.indexOf(self.take_data_tab), _translate("Spectrometer", "Take data"))
        self.neon_spectrum.setText(_translate("Spectrometer", "<html><head/><body><p><img src=\":/neonSpectrum/spectrum_pic.jpg\"/></p></body></html>"))
        self.groupBox.setTitle(_translate("Spectrometer", "Calibration settings"))
        self.save_cal_button.setText(_translate("Spectrometer", "Save calibration file"))
        self.load_cal_button.setText(_translate("Spectrometer", "Load calibration file"))
        self.label_9.setText(_translate("Spectrometer", "Neon spectrum .txt file:"))
        self.calibration_spectrum_path.setText(_translate("Spectrometer", "C:/Users/DUTTLAB7/Evan/pandor/data/0000000000000calibration.txt"))
        self.load_calibration_spectrum_button.setText(_translate("Spectrometer", "Choose file"))
        self.spectrum_reset.setText(_translate("Spectrometer", "Reset"))
        self.label_11.setText(_translate("Spectrometer", "Wavelength of peak (nm):"))
        self.peak_input_box.setText(_translate("Spectrometer", "703.24"))
        self.label_8.setText(_translate("Spectrometer", "Pixel location of peak:"))
        self.calibration_button.setText(_translate("Spectrometer", "Enter"))
        self.calibration_reset.setText(_translate("Spectrometer", "Reset"))
        self.label_10.setText(_translate("Spectrometer", "Background noise file:"))
        self.select_background_button.setText(_translate("Spectrometer", "Choose file"))
        self.reset_background_button.setText(_translate("Spectrometer", "Reset"))
        self.label_16.setText(_translate("Spectrometer", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'MS Shell Dlg 2\'; font-size:8pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:6px; margin-bottom:6px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">To calibrate:</p>\n"
"<ol style=\"margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; -qt-list-indent: 1;\"><li style=\" margin-top:12px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">Load the data file of the neon spectrum you took. Press Enter.</li>\n"
"<li style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">Locate the 703.24 peak in your spectrum using the figure to the left.<br>The pixel location of each peak is shown in red next to the peak in the graph<br> to the right.</li>\n"
"<li style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">Type the location of the peak (in pixels) in the &quot;Pixel location&quot; box. Press Enter.</li>\n"
"<li style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">Check that the calibration is accurate using the two graphs on the bottom.</li>\n"
"<li style=\" margin-top:0px; margin-bottom:12px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">(Optional) upload a background noise file that will be automatically subtracted<br>from all images.</li></ol>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>"))
        self.tab_selector.setTabText(self.tab_selector.indexOf(self.calibration_tab), _translate("Spectrometer", "Calibration"))
        self.maximize_button.setText(_translate("Spectrometer", "Maximize"))
        self.select_display_data_button.setText(_translate("Spectrometer", "Choose file"))
        self.add_display_data_button.setText(_translate("Spectrometer", "Load file"))
        self.reset_display_button.setText(_translate("Spectrometer", "Reset"))
        self.save_data_button.setText(_translate("Spectrometer", "Save Data"))
        self.select_cal_file_button.setText(_translate("Spectrometer", "Choose calibration file"))
        self.reset_display_cal_button.setText(_translate("Spectrometer", "Reset"))
        self.label_17.setText(_translate("Spectrometer", "Minimum y:"))
        self.label_19.setText(_translate("Spectrometer", "Maximum y:"))
        self.tab_selector.setTabText(self.tab_selector.indexOf(self.display_data_tab), _translate("Spectrometer", "Display data"))
from Widgets.mplwidget import MplWidget
from Widgets.qled import QLed
#from NewSpectrometer import neon_pic_rc


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Spectrometer = QtWidgets.QMainWindow()
    ui = Ui_Spectrometer()
    ui.setupUi(Spectrometer)
    Spectrometer.show()
    sys.exit(app.exec_())