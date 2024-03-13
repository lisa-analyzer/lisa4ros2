#!/usr/bin/env python3

#  Telemetry Project:
#  Renato and Michael.

# -------------IMPORTS-------------------
import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import struct
import smbus
from time import time
import os

from telemetry_interfaces.msg import (
    Bool, Int, Battery, UPSSettings, SamplingRate, EnableRead
)

from telemetry_pkg.utilities import RepeatedTimer, TelemetryLogger, generate_str_timestamp, log_msg

# -------------UPS Parameters-------------------
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# Charging (Yes/No)
PLD_PIN = 6
GPIO.setup(PLD_PIN, GPIO.IN)

#  Battery capacity and voltage
GPIO_PORT 	= 26
I2C_ADDR    = 0x36
GPIO.setup(GPIO_PORT, GPIO.OUT)
bus = smbus.SMBus(1) # 0 = /dev/i2c-0 (port I2C0), 1 = /dev/i2c-1 (port I2C1)

# -------------GLOBAL PARAMETERS-----------
PUBLISH_PERIOD = 0.05 # sec
SENSOR_SETTINGS_PERIOD = 2 # sec
PERIOD_INIT_SENSORS = 500 # msec
ENABLE_INIT_SENSORS = True
SENSORS = ["ups_charging", "ups_battery"]

class UpsReadingNode(Node):
    def __init__(self):
        super().__init__("ups_reading")

        self.init_log()

        # init charging and ups
        self.period_sensor_dict = {}
        self.enable_sensor_dict = {}
        self.sensor_last_sample_time_dict = {}
        for sen in SENSORS:
            self.period_sensor_dict[sen] = PERIOD_INIT_SENSORS
            self.enable_sensor_dict[sen] = ENABLE_INIT_SENSORS
            self.sensor_last_sample_time_dict[sen] = 0

        # Publishers
        self.read_ups_charging_publisher_ = self.create_publisher(Bool, "read_ups_charging", 10)
        self.read_ups_battery_publisher_ = self.create_publisher(Battery, "read_ups_battery", 10)
        self.data_ups_publisher_ = self.create_publisher(UPSSettings, "ups_sensor_settings", 10)

        # Sampling Period msec
        self.period_ups_subscriber_ = self.create_subscription(SamplingRate, "set_period", self.callback_set_ups_period, 10)

        # Enable sensor reading
        self.enable_ups_subscriber_ = self.create_subscription(EnableRead, "enable_read", self.callback_enable_ups_read, 10)
        
        self.logger.info("Started UPS node")

    # -------------LOGGER------------
    def init_log(self):
        log_dir = os.path.join(os.path.expanduser('~'), "ros2_ws/telemetry_logs")
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        log_file_path = os.path.join(log_dir, f"ups_reading_log_{generate_str_timestamp()}.txt")

        self.logger = TelemetryLogger('logger', log_file_path).logger
        self.log_level_subscriber_ = self.create_subscription(Int, "log_level", self.callback_set_log_level, 10)

        self.telemetry_data_logger = TelemetryLogger('telemetry_data_logger', log_file_path, console=False, disabled=True).logger                
        self.telemetry_data_log_enable_subscriber_ = self.create_subscription(Bool, "telemetry_data_log_enable", self.callback_telemetry_data_log_enable,10)

    def callback_set_log_level(self, msg):
        self.logger.setLevel(msg.data)

    def callback_telemetry_data_log_enable(self, msg):
        if msg.data:
            self.telemetry_data_logger.disabled = False
        else:
            self.telemetry_data_logger.disabled = True

    def publish(self):
        if self.enable_sensor_dict["ups_charging"]:
            reading = self.ups_charging_read()
            if reading is not None:
                msg = Bool()
                msg.data = reading
                self.read_ups_charging_publisher_.publish(msg)
                log_msg(self.telemetry_data_logger, "published", msg)

        if self.enable_sensor_dict["ups_battery"]:
            reading = self.ups_battery_read()
            if reading is not None:
                msg = Battery()
                msg.voltage = round(reading[0], 2)
                msg.capacity = int(reading[1])
                self.read_ups_battery_publisher_.publish(msg)
                log_msg(self.telemetry_data_logger, "published", msg)

    def ups_charging_read(self):
        if (time() * 1000 - self.sensor_last_sample_time_dict["ups_charging"]) >= self.period_sensor_dict["ups_charging"]:
            self.sensor_last_sample_time_dict["ups_charging"] = time() * 1000
            return(not bool(GPIO.input(PLD_PIN)))
        return None

    def ups_battery_read(self):
        if (time() * 1000 - self.sensor_last_sample_time_dict["ups_battery"]) >= self.period_sensor_dict["ups_battery"]:
            self.sensor_last_sample_time_dict["ups_battery"] = time() * 1000

            read = bus.read_word_data(I2C_ADDR, 2)
            swapped = struct.unpack("<H", struct.pack(">H", read))[0]
            voltage = swapped * 1.25 /1000/16

            read = bus.read_word_data(I2C_ADDR, 4)
            swapped = struct.unpack("<H", struct.pack(">H", read))[0]
            capacity = swapped/256

            return voltage, capacity
        
        return None

    def callback_set_ups_period(self, msg):
        if msg.period_msec <= 0:
            self.logger.warn("Sampling period must be positive")
            return

        if msg.sensor not in SENSORS:
            self.logger.warn(f"Recieved an invalid sensor in set period: {msg.sensor}")
            return
        self.period_sensor_dict[msg.sensor] = msg.period_msec

        log_msg(self.telemetry_data_logger, "recieved", msg)

    def callback_enable_ups_read(self, msg):
        if msg.sensor not in SENSORS:
            self.logger.warn(f"Recieved an invalid sensor in set enable: {msg.sensor}")
            return
        self.enable_sensor_dict[msg.sensor] = bool(msg.enable_read)

        log_msg(self.telemetry_data_logger, "recieved", msg)

    def sensor_settings_publish(self):
        msg = UPSSettings()
        msg.charging = [self.period_sensor_dict['ups_charging'], int(self.enable_sensor_dict['ups_charging'])]
        msg.battery = [self.period_sensor_dict['ups_battery'], int(self.enable_sensor_dict['ups_battery'])]
        self.data_ups_publisher_.publish(msg)
        log_msg(self.telemetry_data_logger, "published", msg)


def main(args=None):
    rclpy.init(args=args) # Initialize ROS2 libraries
    node = UpsReadingNode()
    try:
        publish_timer = RepeatedTimer(PUBLISH_PERIOD, node.publish)
        sensor_settings_timer = RepeatedTimer(SENSOR_SETTINGS_PERIOD, node.sensor_settings_publish)
        rclpy.spin(node) # Make node run in background

    except KeyboardInterrupt:
        node.logger.info("UPS got Keyboard Interrupt")
        pass

    finally: 
        publish_timer.stop()
        sensor_settings_timer.stop()
        node.logger.info("Shutting down UPS")
        node.destroy_node()


if __name__ == "__main__":
    main()