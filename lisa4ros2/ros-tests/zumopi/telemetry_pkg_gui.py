#!/usr/bin/env python3

#  Telemetry Project:
#  Renato and Michael.

# -------------IMPORTS-------------------
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from threading import Thread
from math import pi
import sys
from time import time, sleep
import cv2
import numpy as np
import os

from telemetry_interfaces.msg import (
    Int, Bool, String, Buzzer, PlayBuzzer, Imu, Encoders, LineSensors, ProximitySensors, Motors, Zumo, \
    Battery, UPSSettings, CompressedImage, CameraSettings, SamplingRate, EnableRead, WifiStats
)

from PyQt5.QtCore import QTimer, QRect
from PyQt5.QtWidgets import QApplication, QMainWindow, QComboBox
from PyQt5.QtGui import QPixmap, QImage

from telemetry_pkg.telemetry_ui import Ui_MainWindow
from telemetry_pkg.joystick import Joystick
from telemetry_pkg.utilities import RepeatedTimer, TelemetryLogger, generate_str_timestamp, log_msg

# -------------GLOBAL PARAMETERS-----------
GUI_REFRESH_PERIOD = 10 # msec
ARDUINO_DELAY = 50 # msec minimum sampling rate in spinbox, must be bigger than loop delay on Arduino
UPS_MIN_PERIOD = 50 # msec minimum sampling rate in spinbox
KEEP_ALIVE_PERIOD = 2 # sec
# default values of sampling and enable
PERIOD_RESET_SENSORS = 500 # msec
ENABLE_RESET_SENSORS = True 
PERIOD_RESET_CAMERA = 100 # msec

CAMERA_MIN_PERIOD = 50 # msec -> 20 fps

FRAME_WIDTH = 320
FRAME_HEIGHT = 240
NUM_LAST_SAVED_FRAMES_TIME = 10 # frames saved for fps calculation

KEEP_ALIVE_COUNTER_MAX = 3 # maximum tolerance of not getting signals from nodes
MAX_SPEED_MOTORS = 400
SPEED_MOTORS = 300
MOTORS_PUBLISH_PERIOD = 0.2 # seconds

DEFAULT_LOG = 'INFO'

# ENCODERS PARAMS
GEAR_RATIO = 75.81
WHEEL_RADIUS = 37.5 * 0.001 /2
ENCODER_PPR = 12
ENCODER_CORRECTION = 0.68
ENCODER2DIST = WHEEL_RADIUS*2*pi/(ENCODER_PPR*GEAR_RATIO) * ENCODER_CORRECTION

GOOD_RSSI = -45
BAD_RSSI = -60
MAX_RSSI = -30
MIN_RSSI = -66

APPS = ["connection", "serial_interface", "zumo", "ups", "camera_node", "camera"]
SENSORS = ["buzzer", "imu", "encoders", "line_sensors", "proximity_sensors", "ups_charging", "ups_battery", "camera"]


class GUINode(Node, Ui_MainWindow):
# @@@@@@@@@@@@@@@@@@INIT@@@@@@@@@@@@@@@@@@@@
    def __init__(self, ui):
        super().__init__("gui")
        
        # init period and enable
        self.period_sensor_dict = {}
        self.enable_sensor_dict = {}
        for sen in SENSORS:
            self.period_sensor_dict[sen] = PERIOD_RESET_SENSORS
            self.enable_sensor_dict[sen] = False
        self.period_sensor_dict["camera"] = PERIOD_RESET_CAMERA

        self.period_serial_publisher_ = self.create_publisher(SamplingRate, "set_period", 10)
        self.enable_serial_publisher_ = self.create_publisher(EnableRead, "enable_read", 10)

        # init apps
        self.app_alive_dict = {}
        self.app_keep_alive_time_dict = {}
        self.app_alive_counter = {}
        self.app_status_dict = {}
        self.app_color_dict = {}
        for app in APPS:
            self.app_alive_dict[app] = False
            self.app_keep_alive_time_dict[app] = 0
            self.app_status_dict[app] = f'{app.replace("_", " ").title()} is Down...'
            self.app_color_dict[app] = "red"
            self.app_alive_counter[app] = 0

        # ------------ALL INITS-------------
        self.ui = ui
        self.init_zumo_image()
        self.init_log()
        self.init_wifi_stats()
        self.init_buzzer()
        self.init_imu()
        self.init_encoders()
        self.init_line_sensors()
        self.init_proximity_sensors()
        self.init_motors()
        self.init_charging()
        self.init_battery()
        self.init_camera()

        self.data_zumo_subscriber_ = self.create_subscription(Zumo, "zumo_sensor_settings", self.callback_settings_zumo, 10)
        self.data_ups_subscriber_ = self.create_subscription(UPSSettings, "ups_sensor_settings", self.callback_settings_ups, 10)
        self.data_camera_subscriber_ = self.create_subscription(CameraSettings, "camera_sensor_settings", self.callback_settings_camera, 10)

        self.keep_alive_subscriber_ = self.create_subscription(String, "keep_alive", self.callback_keep_alive, 10)

        self.logger.info("Started GUI node")

# ----------------ZUMO IMAGE-------------------
    def init_zumo_image(self):
        img_dir = os.path.join(os.path.expanduser('~'), "ros2_ws/src/telemetry_pkg/telemetry_pkg/zumo_img.PNG")
        h =  self.ui.zumo_image_frame.frameGeometry().height()
        w = self.ui.zumo_image_frame.frameGeometry().width()
        self.ui.zumo_image_frame.setPixmap(QPixmap(img_dir).scaled(w,h))

# ----------------LOGGING-------------------
    def init_log(self):
        log_dir = os.path.join(os.path.expanduser('~'), "ros2_ws/telemetry_logs")
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        log_file_path = os.path.join(log_dir, f"data_log_{generate_str_timestamp()}.txt")

        self.logger = TelemetryLogger('logger', log_file_path).logger
        self.log_level_publisher_ = self.create_publisher(Int, "log_level", 10)

        self.telemetry_data_logger = TelemetryLogger('telemetry_data_logger', log_file_path, console=False, disabled=True).logger                
        self.telemetry_data_log_enable_publisher_ = self.create_publisher(Bool, "telemetry_data_log_enable", 10)

        self.init_log_level() # used for GUI defenition

    def init_log_level(self):

        self.log_level_map = {
            'DEBUG': 10,
            'INFO': 20,
            'WARNING': 30,
            'ERROR': 40,
            'CRITICAL': 50,
            'NONE': 60
        }

        level_options = ['DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL', 'NONE']
        self.ui.log_level_options.addItems(level_options)
        self.ui.log_level_options.setSizeAdjustPolicy(QComboBox.AdjustToContents)

        default_index = level_options.index(DEFAULT_LOG) if DEFAULT_LOG in level_options else 1
        self.ui.log_level_options.setCurrentIndex(default_index)
        self.ui.log_level_options.currentIndexChanged.connect(self.set_log_level)

        self.set_log_level()

        self.ui.telemetry_data_checkbox.setText("Save telemetry data")
        self.ui.telemetry_data_checkbox.adjustSize()
        self.ui.telemetry_data_checkbox.setChecked(False)
        self.ui.telemetry_data_checkbox.toggled.connect(self.update_save_telemetry_data)

        self.update_save_telemetry_data()

    # set and publish log level
    def set_log_level(self):
        selected_option = self.ui.log_level_options.currentText()
        log_level = self.log_level_map.get(selected_option)
        self.logger.setLevel(log_level)

        msg = Int()
        msg.data = log_level
        self.log_level_publisher_.publish(msg)

    # set and publish if need to save telemetry data
    def update_save_telemetry_data(self):
        if self.ui.telemetry_data_checkbox.isChecked():
            self.telemetry_data_logger.disabled = False
        else:
            self.telemetry_data_logger.disabled = True
        
        msg = Bool()
        msg.data = self.ui.telemetry_data_checkbox.isChecked()
        self.telemetry_data_log_enable_publisher_.publish(msg)

# ----------------WIFI STATS-------------------
    def init_wifi_stats(self):
        # UI init
        self.wifi_stats_rx_tx_rate_color = "transparent"
        self.wifi_stats_signal_power_color = "transparent"
        self.wifi_stats_signal_power = -999
        self.wifi_stats_rx_rate = -1
        self.wifi_stats_tx_rate = -1

        self.ui.signal_power_label.adjustSize()
        self.ui.rx_rate_label.adjustSize()
        self.ui.tx_rate_label.adjustSize()

        self.wifi_stats_signal_power_bar = 0
        self.ui.signal_power_bar.setRange(0,100)

        # ROS2 init
        self.wifi_stats_subscriber_ = self.create_subscription(WifiStats, "wifi_stats", self.callback_wifi_stats, 10)

    def callback_wifi_stats(self, msg):
        self.wifi_stats_signal_power = msg.signal_power
        self.wifi_stats_rx_rate = msg.rx_rate
        self.wifi_stats_tx_rate = msg.tx_rate

        self.wifi_stats_rx_tx_rate_color = "black"

        if self.wifi_stats_signal_power >= GOOD_RSSI:
            self.wifi_stats_signal_power_color = "green"
        elif BAD_RSSI <= self.wifi_stats_signal_power < GOOD_RSSI:
            self.wifi_stats_signal_power_color = "orange"
        else:
            self.wifi_stats_signal_power_color = "red"

        # RSSI BAR PERCENTAGE
        if self.wifi_stats_signal_power >= MAX_RSSI:
            self.wifi_stats_signal_power_bar = 100
        elif self.wifi_stats_signal_power <= MIN_RSSI:
            self.wifi_stats_signal_power_bar = 0
        else:
            self.wifi_stats_signal_power_bar = 100 * (MIN_RSSI - self.wifi_stats_signal_power ) / (MIN_RSSI - MAX_RSSI)

        # If recieved data, it means there is wifi connections
        self.app_keep_alive_time_dict["connection"] = time()
        log_msg(self.telemetry_data_logger, "recieved", msg)


# ----------------ZUMO-------------------
    # Received sensor data from arduino
    def callback_settings_zumo(self, msg):
        self.period_sensor_dict["buzzer"] = msg.buzzer[0]
        self.enable_sensor_dict["buzzer"] = bool(msg.buzzer[1])
        if not self.enable_sensor_dict["buzzer"]:
            self.buzzer_color = "transparent"

        self.period_sensor_dict["imu"] = msg.imu[0]
        self.enable_sensor_dict["imu"] = bool(msg.imu[1])
        if not self.enable_sensor_dict["imu"]:
            self.imu_color = "transparent"

        self.period_sensor_dict["encoders"] = msg.encoders[0]
        self.enable_sensor_dict["encoders"] = bool(msg.encoders[1])
        if not self.enable_sensor_dict["encoders"]:
            self.encoders_left_color = "transparent"
            self.encoders_right_color = "transparent"
            self.speed_color = "transparent"

        self.period_sensor_dict["line_sensors"] = msg.line_sensors[0]
        self.enable_sensor_dict["line_sensors"] = bool(msg.line_sensors[1])
        if not self.enable_sensor_dict["line_sensors"]:
            self.line_sensors_color = "transparent"

        self.period_sensor_dict["proximity_sensors"] = msg.proximity_sensors[0]
        self.enable_sensor_dict["proximity_sensors"] = bool(msg.proximity_sensors[1])
        if not self.enable_sensor_dict["proximity_sensors"]:
            self.proximity_sensors_color = "transparent"
            self.proximity_sensors_basic_color = ["transparent", "transparent", "transparent"]

        # If recieved data, both serial_interface node and zumo are alive
        self.app_keep_alive_time_dict["serial_interface"] = time()
        self.app_keep_alive_time_dict["zumo"] = time()
        log_msg(self.telemetry_data_logger, "recieved", msg)

# ----------------BUZZER-------------------
    def init_buzzer(self):
        # UI init
        self.buzzer_color = "transparent"
        self.buzzer_status = ""
        self.buzzer_button_status = "Play Buzzer"
        self.buzzer_period_color = "transparent"
        self.buzzer_enable_color = "transparent"
        self.ui.buzzer_spinbox.setMinimum(ARDUINO_DELAY)
        self.ui.buzzer_spinbox.setValue(self.period_sensor_dict["buzzer"])        
        self.ui.buzzer_button_play.clicked.connect(lambda: self.play_buzzer(False if self.buzzer_status=="Playing" else True))
        self.ui.buzzer_button_period.clicked.connect(lambda: self.set_period("buzzer", self.ui.buzzer_spinbox.value()))
        self.ui.buzzer_button_enable.clicked.connect(lambda: self.enable_sensor("buzzer", not self.enable_sensor_dict["buzzer"]))
        self.ui.buzzer_reset_button.clicked.connect(lambda: self.reset_sensor("buzzer"))

        # ROS2 init
        self.read_buzzer_subscriber_ = self.create_subscription(Buzzer, "read_buzzer", self.callback_read_buzzer, 10)
        self.play_buzzer_publisher_ = self.create_publisher(PlayBuzzer, "play_buzzer", 10)

    def callback_read_buzzer(self, msg):
        if self.enable_sensor_dict["buzzer"]:
            if msg.is_playing:
                self.buzzer_status = "Playing"
                self.buzzer_color = "green"
                self.buzzer_button_status = "Stop Buzzer"
            else:
                self.buzzer_status = "Not Playing"
                self.buzzer_color = "red"
                self.buzzer_button_status = "Play Buzzer"
            log_msg(self.telemetry_data_logger, "recieved", msg)

    def play_buzzer(self, play):
        msg = PlayBuzzer()
        msg.play_buzzer = int(play)
        self.play_buzzer_publisher_.publish(msg)
        log_msg(self.telemetry_data_logger, "published", msg)

# ----------------IMU------------------
    def init_imu(self):
        # UI init
        self.imu_color = "transparent"
        self.accelerometer = [0, 0, 0]
        self.magnetometer = [0, 0, 0]
        self.gyro = [0, 0, 0]
        self.imu_period_color = "transparent"
        self.imu_enable_color = "transparent"
        self.ui.imu_spinbox.setMinimum(ARDUINO_DELAY)
        self.ui.imu_spinbox.setValue(self.period_sensor_dict["imu"])
        self.ui.imu_button_period.clicked.connect(lambda: self.set_period("imu", self.ui.imu_spinbox.value()))
        self.ui.imu_button_enable.clicked.connect(lambda: self.enable_sensor("imu", not self.enable_sensor_dict["imu"]))
        self.ui.imu_reset_button.clicked.connect(lambda: self.reset_sensor("imu"))

        # ROS2 init
        self.read_imu_subscriber_ = self.create_subscription(Imu, "read_imu", self.callback_read_imu, 10)

    # received data is in coordinates [x, y, z]
    def callback_read_imu(self, msg):
        if self.enable_sensor_dict["imu"]:
            if msg.error_init:
                self.accelerometer = ["ERROR INIT", "ERROR INIT", "ERROR INIT"]
                self.magnetometer = ["ERROR INIT", "ERROR INIT", "ERROR INIT"]
                self.gyro = ["ERROR INIT", "ERROR INIT", "ERROR INIT"]
                self.imu_color = "red"
            elif msg.error_read:
                self.accelerometer = ["ERROR", "ERROR", "ERROR"]
                self.magnetometer = ["ERROR", "ERROR", "ERROR"]
                self.gyro = ["ERROR", "ERROR", "ERROR"]
                self.imu_color = "red"
            else:
                self.accelerometer = msg.accelerometer
                self.magnetometer = msg.magnetometer
                self.gyro = msg.gyro
                self.imu_color = "black"
            log_msg(self.telemetry_data_logger, "recieved", msg)

# ---------------ENCODERS-----------------
    def init_encoders(self):
        # UI init
        self.encoders_left_color = "transparent"
        self.encoders_right_color = "transparent"
        self.encoders_left = 0
        self.encoders_right = 0
        self.speed = 0  # in km/h
        self.speed_color = "transparent"        
        self.encoders_period_color = "transparent"
        self.encoders_enable_color = "transparent"
        self.ui.encoders_spinbox.setMinimum(ARDUINO_DELAY)
        self.ui.encoders_spinbox.setValue(self.period_sensor_dict["encoders"])
        self.ui.encoders_button_period.clicked.connect(lambda: self.set_period("encoders", self.ui.encoders_spinbox.value()))
        self.ui.encoders_button_enable.clicked.connect(lambda: self.enable_sensor("encoders", not self.enable_sensor_dict["encoders"]))
        self.ui.encoders_reset_button.clicked.connect(lambda: self.reset_sensor("encoders"))

        # ROS2 init
        self.read_encoders_subscriber_ = self.create_subscription(Encoders, "read_encoders", self.callback_read_encoders, 10)

    def callback_read_encoders(self, msg):
        if self.enable_sensor_dict["encoders"]:
            if msg.error_left:
                self.encoders_left = "ERROR"
                self.encoders_left_color = "red"
            else:
                self.encoders_left = msg.left
                self.encoders_left_color = "black"

            if msg.error_right:
                self.encoders_right = "ERROR"
                self.encoders_right_color = "red"
            else:
                self.encoders_right = msg.right
                self.encoders_right_color = "black"

            if not msg.error_left and not msg.error_right:
                self.speed = round(ENCODER2DIST * (self.encoders_left + self.encoders_right) / (2 * self.period_sensor_dict["encoders"] * 0.001) * 3.6,2)             
                self.speed_color = "black"
            else:
                self.speed = "ERROR"
                self.speed_color = "red"

            log_msg(self.telemetry_data_logger, "recieved", msg)

# ---------------LINE SENSORS-----------------
    def init_line_sensors(self):
        # UI init
        self.line_sensors_color = "transparent"
        self.line_sensors = [-1, -1, -1] # [left sensor, center sensor, right sensor]
        self.line_sensors_period_color = "transparent"
        self.line_sensors_enable_color = "transparent"
        self.ui.line_sensors_spinbox.setMinimum(ARDUINO_DELAY)
        self.ui.line_sensors_spinbox.setValue(self.period_sensor_dict["line_sensors"])
        self.ui.line_sensors_button_period.clicked.connect(lambda: self.set_period("line_sensors", self.ui.line_sensors_spinbox.value()))
        self.ui.line_sensors_button_enable.clicked.connect(lambda: self.enable_sensor("line_sensors", not self.enable_sensor_dict["line_sensors"]))
        self.ui.line_sensors_reset_button.clicked.connect(lambda: self.reset_sensor("line_sensors"))

        # ROS2 init
        self.read_line_sensors_subscriber_ = self.create_subscription(LineSensors, "read_line_sensors", self.callback_read_line_sensors, 10)

    def callback_read_line_sensors(self, msg):
        if self.enable_sensor_dict["line_sensors"]:
            self.line_sensors = msg.data            
            self.line_sensors_color = "black"
            log_msg(self.telemetry_data_logger, "recieved", msg)

# --------------PROXIMITY SENSORS----------------
    def init_proximity_sensors(self):
        # UI init
        self.proximity_sensors_color = "transparent"
        # proximity_sensors_(left/front/right)_sensor = [left_led_value, right_led_value] -> Sensor is RX, and value recieved from leds
        self.proximity_sensors_left_sensor = [0, 0]
        self.proximity_sensors_front_sensor = [0, 0]
        self.proximity_sensors_right_sensor = [0, 0]
        # proximity_sensors_read_basic = [left_sensor, front_sensor, right_sensor] -> data recieved from other sources (not internal leds)
        self.proximity_sensors_basic_color = ["transparent", "transparent", "transparent"]
        self.proximity_sensors_read_basic = ["no interference", "no interference", "no interference"]
        self.proximity_sensors_period_color = "transparent"
        self.proximity_sensors_enable_color = "transparent"
        self.ui.proximity_sensors_spinbox.setMinimum(ARDUINO_DELAY)
        self.ui.proximity_sensors_spinbox.setValue(self.period_sensor_dict["proximity_sensors"])
        self.ui.proximity_sensors_button_period.clicked.connect(lambda: self.set_period("proximity_sensors", self.ui.proximity_sensors_spinbox.value()))
        self.ui.proximity_sensors_button_enable.clicked.connect(lambda: self.enable_sensor("proximity_sensors", not self.enable_sensor_dict["proximity_sensors"]))
        self.ui.proximity_sensors_reset_button.clicked.connect(lambda: self.reset_sensor("proximity_sensors"))

        # ROS2 init
        self.read_proximity_sensors_subscriber_ = self.create_subscription(ProximitySensors, "read_proximity_sensors", self.callback_read_proximity_sensors, 10)

    def callback_read_proximity_sensors(self, msg):
        if self.enable_sensor_dict["proximity_sensors"]:
            self.proximity_sensors_left_sensor = msg.left_sensor
            self.proximity_sensors_front_sensor = msg.front_sensor
            self.proximity_sensors_right_sensor = msg.right_sensor
            self.proximity_sensors_color = "black"

            for idx, basic in enumerate(msg.read_basic):
                if basic == 1:
                    self.proximity_sensors_read_basic[idx] = "interference!"
                    self.proximity_sensors_basic_color[idx] = "red"
                else:
                    self.proximity_sensors_read_basic[idx] = "no interference"
                    self.proximity_sensors_basic_color[idx] = "green"
                
            log_msg(self.telemetry_data_logger, "recieved", msg)

# ----------------MOTORS------------------
    def init_motors(self):
        # UI init
        self.ui.joystick = Joystick(self.ui.motors_frame)
        self.motors_zero = True     # flag to publish motors stop only three times

        # ROS2 init
        self.motors_publisher_ = self.create_publisher(Motors, "motors", 10)

    def set_motors(self):
        x = self.ui.joystick.value_x
        y = self.ui.joystick.value_y

        if x**2 + y**2 <= 0.1**2:  # if joystick in this circle, robot has to stop
            motors_left = 0
            motors_right = 0
            motors_zero = True
        else:
            if abs(x) <= 0.1 and 0.9 <= abs(y) <= 1: # if joystick is in outer circle, give maximum values
                motors_left = MAX_SPEED_MOTORS if y > 0 else -MAX_SPEED_MOTORS
                motors_right = MAX_SPEED_MOTORS if y > 0 else -MAX_SPEED_MOTORS
            else: # joystick is in normal operation position
                motors_left = (y+x)*SPEED_MOTORS   
                motors_right = (y-x)*SPEED_MOTORS
            motors_zero = False     
        
        if not self.motors_zero:
            self.set_motors_publish(motors_left, motors_right)
            if motors_zero is True:  # if transition to idle, publish just in case a few more times
                self.set_motors_publish(motors_left, motors_right)
                self.set_motors_publish(motors_left, motors_right) 

        # Delay of 1 period of set motors in transition from idle to moving
        self.motors_zero = motors_zero

    def set_motors_publish(self, motors_left, motors_right):
        msg = Motors()
        msg.motors_left = int(motors_left)
        msg.motors_right = int(motors_right)
        self.motors_publisher_.publish(msg)
        log_msg(self.telemetry_data_logger, "published", msg)

# ----------------UPS-------------------
    # Received sensor data from UPS
    def callback_settings_ups(self, msg):
        self.period_sensor_dict["ups_charging"] = msg.charging[0]
        self.enable_sensor_dict["ups_charging"] = bool(msg.charging[1])
        if not self.enable_sensor_dict["ups_charging"]:
            self.charging_color = "transparent"

        self.period_sensor_dict["ups_battery"] = msg.battery[0]
        self.enable_sensor_dict["ups_battery"] = bool(msg.battery[1])
        if not self.enable_sensor_dict["ups_battery"]:
            self.battery_color = "transparent"

        # If recieved data, ups node is alive
        self.app_keep_alive_time_dict["ups"] = time()
        log_msg(self.telemetry_data_logger, "recieved", msg)

# ----------------CHARGING-------------------
    def init_charging(self):
        # UI init
        self.charging_color = "transparent"
        self.charging_status = ""
        self.charging_period_color = "transparent"
        self.charging_enable_color = "transparent"
        self.ui.charging_spinbox.setMinimum(UPS_MIN_PERIOD)
        self.ui.charging_spinbox.setValue(self.period_sensor_dict["ups_charging"])
        self.ui.charging_button_period.clicked.connect(lambda: self.set_period("ups_charging", self.ui.charging_spinbox.value()))
        self.ui.charging_button_enable.clicked.connect(lambda: self.enable_sensor("ups_charging", not self.enable_sensor_dict["ups_charging"]))
        self.ui.charging_reset_button.clicked.connect(lambda: self.reset_sensor("ups_charging"))

        # ROS2 init
        self.read_charging_subscriber_ = self.create_subscription(Bool, "read_ups_charging", self.callback_read_charging, 10)        

    def callback_read_charging(self, msg):
        if self.enable_sensor_dict["ups_charging"]:
            if msg.data:
                self.charging_status = "Charging"
                self.charging_color = "green"
            else:
                self.charging_status = "Not Charging"
                self.charging_color = "red"
            log_msg(self.telemetry_data_logger, "recieved", msg)

# ----------------BATTERY-------------------
    def init_battery(self):
        # UI init
        self.battery_color = "transparent"
        self.voltage = -1
        self.capacity = -1
        self.battery_period_color = "transparent"
        self.battery_enable_color = "transparent"
        self.ui.battery_spinbox.setMinimum(UPS_MIN_PERIOD)
        self.ui.battery_spinbox.setValue(self.period_sensor_dict["ups_battery"])
        self.ui.battery_button_period.clicked.connect(lambda: self.set_period("ups_battery", self.ui.battery_spinbox.value()))
        self.ui.battery_button_enable.clicked.connect(lambda: self.enable_sensor("ups_battery", not self.enable_sensor_dict["ups_battery"]))
        self.ui.battery_reset_button.clicked.connect(lambda: self.reset_sensor("ups_battery"))

        # ROS2 init
        self.read_battery_subscriber_ = self.create_subscription(Battery, "read_ups_battery", self.callback_read_battery, 10)

    def callback_read_battery(self, msg):
        if self.enable_sensor_dict["ups_battery"]:
            self.voltage = round(msg.voltage,2)
            self.capacity = msg.capacity
            self.battery_color = "black"
            log_msg(self.telemetry_data_logger, "recieved", msg)

# ----------------CAMERA-------------------
    def init_camera(self):
        # UI init
        self.camera_period_color = "transparent"
        self.camera_enable_color = "transparent"
        init_frame = np.zeros([FRAME_HEIGHT, FRAME_WIDTH,3], dtype=np.uint8)
        encoded_frame = cv2.imencode('.jpg', init_frame, [cv2.IMWRITE_JPEG_QUALITY, 95])[1]
        self.frame = encoded_frame.tolist()
        self.frame_init = self.frame
        # fps parameters
        self.camera_fps = -1
        self.camera_fps_color = "transparent"
        self.last_saved_frames_time_delta = np.zeros(NUM_LAST_SAVED_FRAMES_TIME, dtype=np.float32)
        self.last_saved_frame_ind = 0
        self.last_saved_frame_time = 0

        # Frame capture directory init
        self.frames_dir = os.path.join(os.path.expanduser('~'), "ros2_ws/telemetry_frames")
        if not os.path.exists(self.frames_dir):
            os.makedirs(self.frames_dir)
        self.ui.camera_button_capture.clicked.connect(self.capture_frame)

        # Video recording init
        self.recording = False
        self.stop_recording = False
        self.camera_video_status = "Capture Video"
        self.camera_video_status_color = "light grey"
        self.ui.video_duration_spinbox.setValue(5)

        self.videos_dir = os.path.join(os.path.expanduser('~'), "ros2_ws/telemetry_videos")
        if not os.path.exists(self.videos_dir):
            os.makedirs(self.videos_dir)
        self.ui.camera_button_video.clicked.connect(self.capture_video)

        # UI widgets init
        self.ui.camera_spinbox.setMinimum(CAMERA_MIN_PERIOD)
        self.ui.camera_spinbox.setValue(self.period_sensor_dict["camera"])
        self.ui.camera_button_period.clicked.connect(lambda: self.set_period("camera", self.ui.camera_spinbox.value()))
        self.ui.camera_button_enable.clicked.connect(lambda: self.enable_sensor("camera", not self.enable_sensor_dict["camera"]))
        self.ui.camera_reset_button.clicked.connect(lambda: self.reset_sensor("camera"))
        
        # ROS2 init
        self.read_frames_subscriber_ = self.create_subscription(CompressedImage, "camera_frame", self.callback_read_frames, 10)

    # Received sensor data from camera node
    def callback_settings_camera(self, msg):
        self.period_sensor_dict["camera"] = msg.period_msec
        self.enable_sensor_dict["camera"] = msg.enable_read
        if not self.enable_sensor_dict["camera"]:
            self.frame = self.frame_init
            self.camera_fps_color = "transparent"

        # If recieved data, camera_node is alive
        self.app_keep_alive_time_dict["camera_node"] = time()
        log_msg(self.telemetry_data_logger, "recieved", msg)

    def callback_read_frames(self, msg):
        if self.enable_sensor_dict["camera"]:
            self.frame = msg.compressed_image
            # calc fps
            cur_time = time()
            self.last_saved_frames_time_delta[self.last_saved_frame_ind] = cur_time - self.last_saved_frame_time
            self.last_saved_frame_ind = (self.last_saved_frame_ind + 1) % NUM_LAST_SAVED_FRAMES_TIME
            self.last_saved_frame_time = cur_time
            self.camera_fps_color = "black"
            self.camera_fps = round(1/self.last_saved_frames_time_delta.mean(),2)

    def capture_frame(self):
        self.logger.info("Capture frame")
        recieved_frame = self.decode_frame_from_jpeg_list()
        path = os.path.join(self.frames_dir, f"frame_{generate_str_timestamp()}.jpg")
        cv2.imwrite(path, recieved_frame)
        self.logger.debug("Frame capture is finished")

    def capture_video(self):
        if not self.recording:
            self.logger.info("Start capturing video")

            path = os.path.join(self.videos_dir, f"video_{generate_str_timestamp()}.mp4")

            if self.camera_fps < 0.1:
                self.logger.warning(f"Cant capture video. Camera fps is {self.camera_fps}")
                return

            thread = Thread(target=self.save_video, args=(path,))
            thread.start()

            self.recording = True
            self.stop_recording = False
            self.camera_video_status = "Stop Recording"
            self.camera_video_status_color = "red"
        else:
            self.stop_recording = True

    def save_video(self, path):
        video = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*'MJPG'), float(self.camera_fps), (FRAME_WIDTH, FRAME_HEIGHT))
        for frame_count in range(round(self.camera_fps*self.ui.video_duration_spinbox.value())):
            if self.stop_recording or not self.enable_sensor_dict["camera"]:
                break

            recieved_frame = self.decode_frame_from_jpeg_list()
            video.write(recieved_frame)
            sleep(1/self.camera_fps)

        video.release()
        
        self.recording = False
        self.camera_video_status = "Capture Video"
        self.camera_video_status_color = "black"

        self.logger.info("Finished capturing video")

    def decode_frame_from_jpeg_list(self, jpeg_list=None):
        if jpeg_list is None: jpeg_list = self.frame
        jpeg_array = np.array(jpeg_list, dtype=np.uint8)
        return cv2.imdecode(jpeg_array, cv2.IMREAD_COLOR)

# ----------------KEEP ALIVE-------------------
    def callback_keep_alive(self, msg):
        app = msg.data
        if app not in APPS:
            return
        self.app_keep_alive_time_dict[app] = time()
        log_msg(self.telemetry_data_logger, "recieved", msg)

    # check if apps alive and update gui parameters accordingly
    def keep_alive_check(self):
        for app in APPS:
            if (time() - self.app_keep_alive_time_dict[app]) <= 1.2*KEEP_ALIVE_PERIOD:
                self.app_alive_dict[app] = True
                self.app_alive_counter[app] = KEEP_ALIVE_COUNTER_MAX
                self.app_status_dict[app] = f'{app.replace("_", " ").title()} is Up!'
                self.app_color_dict[app] = "green"
            elif self.app_alive_counter[app]:
                self.app_alive_dict[app] = True
                self.app_alive_counter[app] -= 1
                dots = '.' * (KEEP_ALIVE_COUNTER_MAX - self.app_alive_counter[app])
                self.app_status_dict[app] = f'{app.replace("_", " ").title()} is unstable{dots}'
                self.app_color_dict[app] = "orange"
            else:
                self.app_alive_dict[app] = False
                self.app_status_dict[app] = f'{app.replace("_", " ").title()} is Down...'
                self.app_color_dict[app] = "red"

# ------------PUBLISH PERIOD & ENABLE---------------
    def set_period(self, sensor, period):
        msg = SamplingRate()
        msg.sensor = sensor
        msg.period_msec = period
        self.period_serial_publisher_.publish(msg)        
        log_msg(self.telemetry_data_logger, "published", msg)

    def enable_sensor(self, sensor, enable):
        msg = EnableRead()
        msg.sensor = sensor
        msg.enable_read = int(enable)
        self.enable_serial_publisher_.publish(msg)
        log_msg(self.telemetry_data_logger, "published", msg)

    def reset_sensor(self, sensor):
        self.enable_sensor(sensor, ENABLE_RESET_SENSORS)
        self.set_period(sensor, PERIOD_RESET_SENSORS)

        if sensor=="buzzer":
            self.play_buzzer(False)
        elif sensor=="camera":
            self.set_period("camera", PERIOD_RESET_CAMERA)

# @@@@@@@@@@@@@@@@@@GUI@@@@@@@@@@@@@@@@@@@@@
# ----------------LABELS-------------------
    def refresh_connection_label(self):
        self.ui.connection_label.setText(self.app_status_dict["connection"])
        self.ui.connection_label.setStyleSheet(f"color: {self.app_color_dict['connection']}")
        self.ui.connection_label.adjustSize()

    def refresh_serial_interface_label(self):
        self.ui.serial_interface_label.setText(self.app_status_dict["serial_interface"])
        self.ui.serial_interface_label.setStyleSheet(f"color: {self.app_color_dict['serial_interface']}")
        self.ui.serial_interface_label.adjustSize()

    def refresh_zumo_label(self):
        self.ui.zumo_label.setText(self.app_status_dict["zumo"])
        self.ui.zumo_label.setStyleSheet(f"color: {self.app_color_dict['zumo']}")
        self.ui.zumo_label.adjustSize()

    def refresh_ups_label(self):
        self.ui.ups_label.setText(self.app_status_dict["ups"])
        self.ui.ups_label.setStyleSheet(f"color: {self.app_color_dict['ups']}")
        self.ui.ups_label.adjustSize()

    def refresh_camera_labels(self):
        self.ui.camera_node_label.setText(self.app_status_dict["camera_node"])
        self.ui.camera_node_label.setStyleSheet(f"color: {self.app_color_dict['camera_node']}")
        self.ui.camera_node_label.adjustSize()

        self.ui.camera_label.setText(self.app_status_dict["camera"])
        self.ui.camera_label.setStyleSheet(f"color: {self.app_color_dict['camera']}")
        self.ui.camera_label.adjustSize()

# ----------------WIFI STATS-------------------
    def refresh_wifi_stats(self, status):
        self.wifi_stats_widgets_enable(status)

        self.ui.signal_power_status.setText(f'{self.wifi_stats_signal_power} dBm')
        self.ui.signal_power_status.setStyleSheet(f"color: {self.wifi_stats_signal_power_color}")
        self.ui.signal_power_status.adjustSize()

        self.ui.signal_power_bar.setValue(int(self.wifi_stats_signal_power_bar))
        self.ui.signal_power_bar.setStyleSheet(f"color: {self.wifi_stats_rx_tx_rate_color}")

        self.ui.rx_rate_status.setText(f'{round(self.wifi_stats_rx_rate,2)}')
        self.ui.rx_rate_status.setStyleSheet(f"color: {self.wifi_stats_rx_tx_rate_color}")
        self.ui.rx_rate_label_units.setStyleSheet(f"color: {self.wifi_stats_rx_tx_rate_color}")
        self.ui.rx_rate_status.adjustSize()
        self.ui.tx_rate_status.setText(f'{round(self.wifi_stats_tx_rate,2)}')
        self.ui.tx_rate_status.setStyleSheet(f"color: {self.wifi_stats_rx_tx_rate_color}")
        self.ui.tx_rate_label_units.setStyleSheet(f"color: {self.wifi_stats_rx_tx_rate_color}")
        self.ui.tx_rate_status.adjustSize()

    def wifi_stats_widgets_enable(self, status):
        if not status:
            self.wifi_stats_rx_tx_rate_color = "transparent"
            self.wifi_stats_signal_power_color = "transparent"
            self.wifi_stats_signal_power_bar = 0

# ----------------BUZZER-------------------
    def refresh_buzzer(self, status):
        self.buzzer_widgets_enable(status)
        self.ui.buzzer_label_status.setText(self.buzzer_status)
        self.ui.buzzer_label_status.setStyleSheet(f"color: {self.buzzer_color}")
        self.ui.buzzer_label_status.adjustSize()
        self.ui.buzzer_label_period.setText(f'{self.period_sensor_dict["buzzer"]} msec')
        self.ui.buzzer_label_period.setStyleSheet(f"color: {self.buzzer_period_color}")
        self.ui.buzzer_label_period.adjustSize()
        self.ui.buzzer_label_enable.setText(f'{self.enable_sensor_dict["buzzer"]}')
        self.ui.buzzer_label_enable.setStyleSheet(f"color: {self.buzzer_enable_color}")
        self.ui.buzzer_label_enable.adjustSize()
        self.ui.buzzer_button_play.setText(self.buzzer_button_status if self.enable_sensor_dict["buzzer"] else "Play Buzzer")

    def buzzer_widgets_enable(self, status):
        self.ui.buzzer_button_period.setEnabled(status)
        self.ui.buzzer_button_play.setEnabled(status and self.enable_sensor_dict["buzzer"])
        self.ui.buzzer_reset_button.setEnabled(status)
        self.ui.buzzer_button_enable.setEnabled(status)
        self.ui.buzzer_spinbox.setEnabled(status)
        if not status:
            self.buzzer_button_status = "Play Buzzer"
            self.buzzer_color = "transparent"
            self.buzzer_period_color = "transparent"
            self.buzzer_enable_color = "transparent"
        else:
            self.buzzer_period_color = "black"
            self.buzzer_enable_color = "black"

# -----------------IMU--------------------
    def refresh_imu(self, status):
        self.imu_widgets_enable(status)
        self.ui.imu_accelerometer_x.setText(f'{self.accelerometer[0]}')
        self.ui.imu_accelerometer_x.setStyleSheet(f"color: {self.imu_color}")
        self.ui.imu_accelerometer_x.adjustSize()
        self.ui.imu_accelerometer_y.setText(f'{self.accelerometer[1]}')
        self.ui.imu_accelerometer_y.setStyleSheet(f"color: {self.imu_color}")
        self.ui.imu_accelerometer_y.adjustSize()
        self.ui.imu_accelerometer_z.setText(f'{self.accelerometer[2]}')
        self.ui.imu_accelerometer_z.setStyleSheet(f"color: {self.imu_color}")
        self.ui.imu_accelerometer_z.adjustSize()
        self.ui.imu_magnetometer_x.setText(f'{self.magnetometer[0]}')
        self.ui.imu_magnetometer_x.setStyleSheet(f"color: {self.imu_color}")
        self.ui.imu_magnetometer_x.adjustSize()
        self.ui.imu_magnetometer_y.setText(f'{self.magnetometer[1]}')
        self.ui.imu_magnetometer_y.setStyleSheet(f"color: {self.imu_color}")
        self.ui.imu_magnetometer_y.adjustSize()
        self.ui.imu_magnetometer_z.setText(f'{self.magnetometer[2]}')
        self.ui.imu_magnetometer_z.setStyleSheet(f"color: {self.imu_color}")
        self.ui.imu_magnetometer_z.adjustSize()
        self.ui.imu_gyro_x.setText(f'{self.gyro[0]}')
        self.ui.imu_gyro_x.setStyleSheet(f"color: {self.imu_color}")
        self.ui.imu_gyro_x.adjustSize()
        self.ui.imu_gyro_y.setText(f'{self.gyro[1]}')
        self.ui.imu_gyro_y.setStyleSheet(f"color: {self.imu_color}")
        self.ui.imu_gyro_y.adjustSize()
        self.ui.imu_gyro_z.setText(f'{self.gyro[2]}')
        self.ui.imu_gyro_z.setStyleSheet(f"color: {self.imu_color}")
        self.ui.imu_gyro_z.adjustSize()
        self.ui.imu_label_period.setText(f'{self.period_sensor_dict["imu"]} msec')
        self.ui.imu_label_period.setStyleSheet(f"color: {self.imu_period_color}")
        self.ui.imu_label_period.adjustSize()
        self.ui.imu_label_enable.setText(f'{self.enable_sensor_dict["imu"]}')
        self.ui.imu_label_enable.setStyleSheet(f"color: {self.imu_enable_color}")
        self.ui.imu_label_enable.adjustSize()

    def imu_widgets_enable(self, status):
        self.ui.imu_button_period.setEnabled(status)
        self.ui.imu_button_enable.setEnabled(status)
        self.ui.imu_reset_button.setEnabled(status)
        self.ui.imu_spinbox.setEnabled(status)
        if not status:
            self.imu_color = "transparent"
            self.imu_period_color = "transparent"
            self.imu_enable_color = "transparent"
        else:
            self.imu_period_color = "black"
            self.imu_enable_color = "black"

# ---------------ENCODERS-----------------
    def refresh_encoders(self, status):
        self.encoders_widgets_enable(status)
        self.ui.encoders_left.setText(f'{self.encoders_left}')
        self.ui.encoders_left.setStyleSheet(f"color: {self.encoders_left_color}")
        self.ui.encoders_left.adjustSize()
        self.ui.encoders_right.setText(f'{self.encoders_right}')
        self.ui.encoders_right.setStyleSheet(f"color: {self.encoders_right_color}")
        self.ui.encoders_right.adjustSize()

        self.ui.speed.setText(f'{self.speed}')
        self.ui.speed.setStyleSheet(f"color: {self.speed_color}")
        self.ui.speed_units.setStyleSheet(f"color: {self.speed_color}")
        self.ui.speed.adjustSize()

        self.ui.encoders_label_period.setText(f'{self.period_sensor_dict["encoders"]} msec')
        self.ui.encoders_label_period.setStyleSheet(f"color: {self.encoders_period_color}")
        self.ui.encoders_label_period.adjustSize()
        self.ui.encoders_label_enable.setText(f'{self.enable_sensor_dict["encoders"]}')
        self.ui.encoders_label_enable.setStyleSheet(f"color: {self.encoders_enable_color}")
        self.ui.encoders_label_enable.adjustSize()

    def encoders_widgets_enable(self, status):
        self.ui.encoders_button_period.setEnabled(status)
        self.ui.encoders_button_enable.setEnabled(status)
        self.ui.encoders_reset_button.setEnabled(status)
        self.ui.encoders_spinbox.setEnabled(status)
        if not status:
            self.encoders_left_color = "transparent"
            self.encoders_right_color = "transparent"
            self.speed_color = "transparent"
            self.encoders_period_color = "transparent"
            self.encoders_enable_color = "transparent"
        else:
            self.encoders_period_color = "black"
            self.encoders_enable_color = "black"

# ---------------LINE SENSORS-----------------
    def refresh_line_sensors(self, status):
        self.line_sensors_widgets_enable(status)
        self.ui.line_sensor_left.setText(f'{self.line_sensors[0]}')
        self.ui.line_sensor_left.setStyleSheet(f"color: {self.line_sensors_color}")
        self.ui.line_sensor_left.adjustSize()
        self.ui.line_sensor_center.setText(f'{self.line_sensors[1]}')
        self.ui.line_sensor_center.setStyleSheet(f"color: {self.line_sensors_color}")
        self.ui.line_sensor_center.adjustSize()
        self.ui.line_sensor_right.setText(f'{self.line_sensors[2]}')
        self.ui.line_sensor_right.setStyleSheet(f"color: {self.line_sensors_color}")
        self.ui.line_sensor_right.adjustSize()
        self.ui.line_sensors_label_period.setText(f'{self.period_sensor_dict["line_sensors"]} msec')
        self.ui.line_sensors_label_period.setStyleSheet(f"color: {self.line_sensors_period_color}")
        self.ui.line_sensors_label_period.adjustSize()
        self.ui.line_sensors_label_enable.setText(f'{self.enable_sensor_dict["line_sensors"]}')
        self.ui.line_sensors_label_enable.setStyleSheet(f"color: {self.line_sensors_enable_color}")
        self.ui.line_sensors_label_enable.adjustSize()

    def line_sensors_widgets_enable(self, status):
        self.ui.line_sensors_button_period.setEnabled(status)
        self.ui.line_sensors_button_enable.setEnabled(status)
        self.ui.line_sensors_reset_button.setEnabled(status)
        self.ui.line_sensors_spinbox.setEnabled(status)
        if not status:
            self.line_sensors_color = "transparent"
            self.line_sensors_period_color = "transparent"
            self.line_sensors_enable_color = "transparent"
        else:
            self.line_sensors_period_color = "black"
            self.line_sensors_enable_color = "black"

# --------------PROXIMITY SENSORS----------------
    def refresh_proximity_sensors(self, status):
        self.proximity_sensors_widgets_enable(status)
        self.ui.proximity_sensors_tx_left_rx_left.setText(f'{self.proximity_sensors_left_sensor[0]}')
        self.ui.proximity_sensors_tx_left_rx_left.setStyleSheet(f"color: {self.proximity_sensors_color}")
        self.ui.proximity_sensors_tx_left_rx_left.adjustSize()
        self.ui.proximity_sensors_tx_left_rx_front.setText(f'{self.proximity_sensors_front_sensor[0]}')
        self.ui.proximity_sensors_tx_left_rx_front.setStyleSheet(f"color: {self.proximity_sensors_color}")
        self.ui.proximity_sensors_tx_left_rx_front.adjustSize()
        self.ui.proximity_sensors_tx_left_rx_right.setText(f'{self.proximity_sensors_right_sensor[0]}')
        self.ui.proximity_sensors_tx_left_rx_right.setStyleSheet(f"color: {self.proximity_sensors_color}")
        self.ui.proximity_sensors_tx_left_rx_right.adjustSize()
        self.ui.proximity_sensors_tx_right_rx_left.setText(f'{self.proximity_sensors_left_sensor[1]}')
        self.ui.proximity_sensors_tx_right_rx_left.setStyleSheet(f"color: {self.proximity_sensors_color}")
        self.ui.proximity_sensors_tx_right_rx_left.adjustSize()
        self.ui.proximity_sensors_tx_right_rx_front.setText(f'{self.proximity_sensors_front_sensor[1]}')
        self.ui.proximity_sensors_tx_right_rx_front.setStyleSheet(f"color: {self.proximity_sensors_color}")
        self.ui.proximity_sensors_tx_right_rx_front.adjustSize()
        self.ui.proximity_sensors_tx_right_rx_right.setText(f'{self.proximity_sensors_right_sensor[1]}')
        self.ui.proximity_sensors_tx_right_rx_right.setStyleSheet(f"color: {self.proximity_sensors_color}")
        self.ui.proximity_sensors_tx_right_rx_right.adjustSize()
        self.ui.proximity_sensors_tx_basic_rx_left.setText(f'{self.proximity_sensors_read_basic[0]}')
        self.ui.proximity_sensors_tx_basic_rx_left.setStyleSheet(f"color: {self.proximity_sensors_basic_color[0]}")
        self.ui.proximity_sensors_tx_basic_rx_left.adjustSize()
        self.ui.proximity_sensors_tx_basic_rx_front.setText(f'{self.proximity_sensors_read_basic[1]}')
        self.ui.proximity_sensors_tx_basic_rx_front.setStyleSheet(f"color: {self.proximity_sensors_basic_color[1]}")
        self.ui.proximity_sensors_tx_basic_rx_front.adjustSize()
        self.ui.proximity_sensors_tx_basic_rx_right.setText(f'{self.proximity_sensors_read_basic[2]}')
        self.ui.proximity_sensors_tx_basic_rx_right.setStyleSheet(f"color: {self.proximity_sensors_basic_color[2]}")
        self.ui.proximity_sensors_tx_basic_rx_right.adjustSize()
        self.ui.proximity_sensors_label_period.setText(f'{self.period_sensor_dict["proximity_sensors"]} msec')
        self.ui.proximity_sensors_label_period.setStyleSheet(f"color: {self.proximity_sensors_period_color}")
        self.ui.proximity_sensors_label_period.adjustSize()
        self.ui.proximity_sensors_label_enable.setText(f'{self.enable_sensor_dict["proximity_sensors"]}')
        self.ui.proximity_sensors_label_enable.setStyleSheet(f"color: {self.proximity_sensors_enable_color}")
        self.ui.proximity_sensors_label_enable.adjustSize()

    def proximity_sensors_widgets_enable(self, status):
        self.ui.proximity_sensors_button_period.setEnabled(status)
        self.ui.proximity_sensors_button_enable.setEnabled(status)
        self.ui.proximity_sensors_reset_button.setEnabled(status)
        self.ui.proximity_sensors_spinbox.setEnabled(status)
        if not status:            
            self.proximity_sensors_color = "transparent"
            self.proximity_sensors_basic_color = ["transparent", "transparent", "transparent"]
            self.proximity_sensors_period_color = "transparent"
            self.proximity_sensors_enable_color = "transparent"
        else:
            self.proximity_sensors_period_color = "black"
            self.proximity_sensors_enable_color = "black"

# ----------------MOTORS-------------------
    def refresh_motors(self, status):
        self.ui.joystick.setEnabled(status)

# ----------------CHARGING-------------------
    def refresh_charging(self, status):
        self.charging_widgets_enable(status)
        self.ui.charging_label_status.setText(self.charging_status)
        self.ui.charging_label_status.setStyleSheet(f"color: {self.charging_color}")
        self.ui.charging_label_status.adjustSize()
        self.ui.charging_label_period.setText(f'{self.period_sensor_dict["ups_charging"]} msec')
        self.ui.charging_label_period.setStyleSheet(f"color: {self.charging_period_color}")
        self.ui.charging_label_period.adjustSize()
        self.ui.charging_label_enable.setText(f'{self.enable_sensor_dict["ups_charging"]}')
        self.ui.charging_label_enable.setStyleSheet(f"color: {self.charging_enable_color}")
        self.ui.charging_label_enable.adjustSize()

    def charging_widgets_enable(self, status):
        self.ui.charging_button_period.setEnabled(status)
        self.ui.charging_button_enable.setEnabled(status)
        self.ui.charging_reset_button.setEnabled(status)
        self.ui.charging_spinbox.setEnabled(status)
        if not status:
            self.charging_color = "transparent"
            self.charging_period_color = "transparent"
            self.charging_enable_color = "transparent"
        else:
            self.charging_period_color = "black"
            self.charging_enable_color = "black"

# ----------------BATTERY-------------------
    def refresh_battery(self, status):
        self.battery_widgets_enable(status)
        self.ui.battery_voltage.setText(f'{self.voltage} [V]')
        self.ui.battery_voltage.setStyleSheet(f"color: {self.battery_color}")
        self.ui.battery_voltage.adjustSize()
        self.ui.battery_capacity.setText(f'{self.capacity} %')
        self.ui.battery_capacity.setStyleSheet(f"color: {self.battery_color}")
        self.ui.battery_capacity.adjustSize()
        self.ui.battery_label_period.setText(f'{self.period_sensor_dict["ups_battery"]} msec')
        self.ui.battery_label_period.setStyleSheet(f"color: {self.battery_period_color}")
        self.ui.battery_label_period.adjustSize()
        self.ui.battery_label_enable.setText(f'{self.enable_sensor_dict["ups_battery"]}')
        self.ui.battery_label_enable.setStyleSheet(f"color: {self.battery_enable_color}")
        self.ui.battery_label_enable.adjustSize()

    def battery_widgets_enable(self, status):
        self.ui.battery_button_period.setEnabled(status)
        self.ui.battery_button_enable.setEnabled(status)
        self.ui.battery_reset_button.setEnabled(status)
        self.ui.battery_spinbox.setEnabled(status)
        if not status:
            self.battery_color = "transparent"
            self.battery_period_color = "transparent"
            self.battery_enable_color = "transparent"
        else:
            self.battery_period_color = "black"
            self.battery_enable_color = "black"

# ----------------CAMERA-------------------
    def refresh_camera(self,status):
        self.camera_widgets_enable(status)

        recieved_frame = self.decode_frame_from_jpeg_list()
        w,h,_ = recieved_frame.shape
        qimg = QImage(recieved_frame.data, h, w, 3*h, QImage.Format_BGR888)
        self.ui.camera_frame.setPixmap(QPixmap(qimg))

        self.ui.camera_frame.adjustSize()
        self.ui.camera_label_period.setText(f'{self.period_sensor_dict["camera"]} msec')
        self.ui.camera_label_period.setStyleSheet(f"color: {self.camera_period_color}")
        self.ui.camera_label_period.adjustSize()
        self.ui.camera_label_enable.setText(f'{self.enable_sensor_dict["camera"]}')
        self.ui.camera_label_enable.setStyleSheet(f"color: {self.camera_enable_color}")
        self.ui.camera_label_enable.adjustSize()
        self.ui.camera_fps.setText(f"{self.camera_fps}")
        self.ui.camera_fps.setStyleSheet(f"color: {self.camera_fps_color}")
        self.ui.camera_fps.adjustSize()

        self.ui.camera_button_video.setText(self.camera_video_status)
        self.ui.camera_button_video.setStyleSheet(f"color: {self.camera_video_status_color}")

    def camera_widgets_enable(self, status):
        self.ui.camera_button_period.setEnabled(status)
        self.ui.camera_button_enable.setEnabled(status)
        self.ui.camera_reset_button.setEnabled(status)
        self.ui.camera_spinbox.setEnabled(status)
        self.ui.camera_button_capture.setEnabled(status and self.enable_sensor_dict["camera"])
        self.ui.camera_button_video.setEnabled(status and self.enable_sensor_dict["camera"])
        self.ui.video_duration_spinbox.setEnabled(status and self.enable_sensor_dict["camera"])
        if not status:
            self.camera_period_color = "transparent"
            self.camera_enable_color = "transparent"
            self.camera_fps_color = "transparent"
            self.frame = self.frame_init
            self.camera_video_status_color = "light grey"
            self.stop_recording = True
        else:
            self.camera_period_color = "black"
            self.camera_enable_color = "black"
            if not self.enable_sensor_dict["camera"]: self.camera_video_status_color = "light grey"

# --------------GUI REFRESH-----------------
    def refresh_gui_data(self):
        self.refresh_connection_label()
        self.refresh_serial_interface_label()
        self.refresh_zumo_label()
        self.refresh_ups_label()
        self.refresh_camera_labels()

        self.refresh_wifi_stats(self.app_alive_dict["connection"])

        zumo_intf_enable = self.app_alive_dict["connection"] and self.app_alive_dict["serial_interface"] and self.app_alive_dict["zumo"]
        self.refresh_buzzer(zumo_intf_enable)
        self.refresh_imu(zumo_intf_enable)
        self.refresh_encoders(zumo_intf_enable)
        self.refresh_line_sensors(zumo_intf_enable)
        self.refresh_proximity_sensors(zumo_intf_enable)
        self.refresh_motors(zumo_intf_enable)

        ups_enable = self.app_alive_dict["connection"] and self.app_alive_dict["ups"]
        self.refresh_charging(ups_enable)
        self.refresh_battery(ups_enable)

        camera_enable = self.app_alive_dict["connection"] and self.app_alive_dict["camera_node"] and self.app_alive_dict["camera"]
        self.refresh_camera(camera_enable)

# ----------------MAIN-------------------
def main(args=None):
    # GUI initialization
    app = QApplication(sys.argv)
    win = QMainWindow()
    gui = Ui_MainWindow()
    gui.setupUi(win)
    
    # Start GUI node
    rclpy.init(args=args)
    node = GUINode(gui)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    thread = Thread(target=executor.spin)
    thread.start()

    # Let the app run on the main thread
    try:
        win.show()
        node.ui.exit_gui_button.clicked.connect(app.exit)
        keep_alive_timer = RepeatedTimer(KEEP_ALIVE_PERIOD, node.keep_alive_check)
        set_motors_timer = RepeatedTimer(MOTORS_PUBLISH_PERIOD, node.set_motors)
        node.timer = QTimer()
        node.timer.setInterval(GUI_REFRESH_PERIOD)
        node.timer.timeout.connect(node.refresh_gui_data)
        node.timer.start()
        sys.exit(app.exec_())

    except:
        # After closing the GUI, it enters here
        pass 
    
    finally:

        if not node.motors_zero: # if not idle, stop motors when exiting
            node.set_motors_publish(0,0)
        if node.recording: # if recording, stop video recording
            node.stop_recording = True
            while node.recording: # let recoding thread finish
                sleep(.1)
        set_motors_timer.stop()
        keep_alive_timer.stop()        
        node.logger.info("Shutting down GUI")
        node.destroy_node()
        executor.shutdown()

if __name__ == '__main__':
    main()
