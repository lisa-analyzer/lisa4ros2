#!/usr/bin/env python3

#  Telemetry Project:
#  Renato and Michael.

# -------------IMPORTS-------------------
import rclpy
from rclpy.node import Node
import serial
import json
from time import time, sleep
import os

from telemetry_interfaces.msg import (
    Bool, Int, String, Buzzer, PlayBuzzer, Imu, Encoders, LineSensors, ProximitySensors, Motors, Zumo, SamplingRate, EnableRead
)

from telemetry_pkg.utilities import RepeatedTimer, TelemetryLogger, generate_str_timestamp, log_msg

# -------------GLOBAL PARAMETERS-----------
PUBLISH_PERIOD = 0.02 # sec
KEEP_ALIVE_PERIOD = 2 # sec
ARDUINO_SENSORS = ["buzzer", "imu", "encoders", "line_sensors", "proximity_sensors"]
ARDUINO_SERIAL_PORT = '/dev/ttyACM0'
ARDUINO_BITRATE = 115200

class SerialInterfaceNode(Node):
    def __init__(self):
        super().__init__("serial_interface")
        self.serial = serial.Serial(ARDUINO_SERIAL_PORT, ARDUINO_BITRATE) # open serial port

        self.init_log()

        # Publishers
        self.read_buzzer_publisher_ = self.create_publisher(Buzzer, "read_buzzer", 10)
        self.read_imu_publisher_ = self.create_publisher(Imu, "read_imu", 10)
        self.read_encoders_publisher_ = self.create_publisher(Encoders, "read_encoders", 10)
        self.read_line_sensors_publisher_ = self.create_publisher(LineSensors, "read_line_sensors", 10)
        self.read_proximity_sensors_publisher_ = self.create_publisher(ProximitySensors, "read_proximity_sensors", 10)
        self.data_zumo_publisher_ = self.create_publisher(Zumo, "zumo_sensor_settings", 10)
        self.keep_alive_publisher_ = self.create_publisher(String, "keep_alive", 10)

        # Subscribers
        self.play_buzzer_subscriber_ = self.create_subscription(PlayBuzzer, "play_buzzer", self.callback_play_buzzer, 10)
        self.motors_subscriber_ = self.create_subscription(Motors, "motors", self.callback_motors, 10)

        # Sampling Period msec
        self.period_serial_subscriber_ = self.create_subscription(SamplingRate, "set_period", self.callback_set_serial_period, 10)

        # Enable sensor reading
        self.enable_serial_subscriber_ = self.create_subscription(EnableRead, "enable_read", self.callback_enable_serial_read, 10)

        # keep alive parametrs
        self.zumo_keep_alive_time = 0 # last time recieved data from zumo
        self.zumo_alive = False

        self.logger.info("Started Serial Interface")

    # -------------LOGGER------------
    def init_log(self):
        log_dir = os.path.join(os.path.expanduser('~'), "ros2_ws/telemetry_logs")
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        log_file_path = os.path.join(log_dir, f"serial_interface_log_{generate_str_timestamp()}.txt")

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

    # Set the sampling rate of Arduino sensors
    def callback_set_serial_period(self, msg):
        if not self.serial.isOpen:
            self.logger.warn("Serial is not open")
            return

        if not self.zumo_alive:
            return
        
        if msg.period_msec <= 0:
            self.logger.warn("Sampling period must be positive")
            return
    
        # not an arduino sensor, don't send
        if msg.sensor not in ARDUINO_SENSORS:
            self.logger.warn(f"Recieved an invalid sensor in set period: {msg.sensor}")
            return

        # write to Arduino
        json_output = {f'period_{msg.sensor}':msg.period_msec}
        json_output = str(json_output) + '\r\n'
        json_output = bytes(json_output, 'utf-8')
        self.serial.write(json_output)

        log_msg(self.telemetry_data_logger, "recieved", msg)


    # Enable/disable the sampling of Arduino sensors
    def callback_enable_serial_read(self, msg):
        if not self.serial.isOpen:
            self.logger.warn("Serial is not open")
            return

        if not self.zumo_alive:
            return

        # not an arduino sensor, don't send
        if msg.sensor not in ARDUINO_SENSORS:
            self.logger.warn(f"Recieved an invalid sensor in set enable: {msg.sensor}")
            return

        # write to Arduino
        json_output = {f'enable_read_{msg.sensor}':msg.enable_read}
        json_output = str(json_output) + '\r\n'
        json_output = bytes(json_output, 'utf-8')
        self.serial.write(json_output)

        log_msg(self.telemetry_data_logger, "recieved", msg)

    def callback_play_buzzer(self, msg):
        if not self.serial.isOpen:
            self.logger.warn("Serial is not open")
            return

        if not self.zumo_alive:
            return

        # write to Arduino
        json_output = {f'play_buzzer':msg.play_buzzer}
        json_output = str(json_output) + '\r\n'
        json_output = bytes(json_output, 'utf-8')
        self.serial.write(json_output)

        log_msg(self.telemetry_data_logger, "recieved", msg)

    def callback_motors(self, msg):
        if not self.serial.isOpen:
            self.logger.warn("Serial is not open")
            return

        if not self.zumo_alive:
            return

        # write to Arduino
        json_output = {f'motors':[msg.motors_left, msg.motors_right]}
        json_output = str(json_output) + '\r\n'
        json_output = bytes(json_output, 'utf-8')
        self.serial.write(json_output)

        log_msg(self.telemetry_data_logger, "recieved", msg)

    # read from serial Arduino data, and publish
    def publish(self):
        json_input = self.read_serial()
        if json_input is None: # did not decode json type from serial
            return

        if "sensor" not in json_input.keys():
            self.logger.warn(f'Recieved a dictionary without a sensor from Arduino: {json_input}')
            return
        
        sensor = json_input["sensor"]
        try:
            if sensor == "buzzer":
                msg = Buzzer()
                msg.is_playing = bool(json_input["is_playing"])
                self.read_buzzer_publisher_.publish(msg)
                log_msg(self.telemetry_data_logger, "published", msg)
            elif sensor == "imu":
                msg = Imu()
                msg.error_read = bool(json_input["error_read"])
                msg.error_init = bool(json_input["error_init"])
                if not msg.error_read and not msg.error_init: # no errors, take data
                    msg.accelerometer = json_input["accelerometer"]
                    msg.magnetometer = json_input["magnetometer"]
                    msg.gyro = json_input["gyro"]
                self.read_imu_publisher_.publish(msg)
                log_msg(self.telemetry_data_logger, "published", msg)
            elif sensor == "encoders":
                msg = Encoders()
                msg.error_right = bool(json_input["error_right"])
                msg.error_left = bool(json_input["error_left"])
                if not msg.error_right: # no errors, take data
                    msg.right = json_input["right"]
                if not msg.error_left: # no errors, take data
                    msg.left = json_input["left"]
                self.read_encoders_publisher_.publish(msg)
                log_msg(self.telemetry_data_logger, "published", msg)
            elif sensor == "line_sensors":
                msg = LineSensors()
                msg.data = json_input["data"]
                self.read_line_sensors_publisher_.publish(msg)
                log_msg(self.telemetry_data_logger, "published", msg)
            elif sensor == "proximity_sensors":
                msg = ProximitySensors()
                msg.left_sensor = json_input["left_sensor"]
                msg.front_sensor = json_input["front_sensor"]
                msg.right_sensor = json_input["right_sensor"]
                msg.read_basic = json_input["read_basic"]
                self.read_proximity_sensors_publisher_.publish(msg)
                log_msg(self.telemetry_data_logger, "published", msg)
            elif sensor == "zumo":
                msg = Zumo()
                msg.buzzer = json_input["buzzer"]
                msg.imu = json_input["imu"]
                msg.encoders = json_input["encoders"]
                msg.line_sensors = json_input["line_sensors"]
                msg.proximity_sensors = json_input["proximity_sensors"]
                self.data_zumo_publisher_.publish(msg)
                log_msg(self.telemetry_data_logger, "published", msg)
                self.zumo_keep_alive_time = time() # if got sensor data from Zumo, it's alive now
            else:
                self.logger.warn(f'Recieved an invalid sensor from Arduino: {json_input}')
        except:
            self.logger.warn(f'Recieved an invalid key in sensor: {sensor}')

    def read_serial(self):  # can read only json type input
        if not self.serial.isOpen:
            self.logger.warn("Serial is not open")
            return None
        
        # use try-except may be a case that serial cannot read and we get errors
        try:
            if self.serial.in_waiting > 0:
                serial_stream = self.serial.readline().decode('utf-8').rstrip('\r\n') # println from arduino
                try:
                    json_input = json.loads(serial_stream)
                    if not isinstance(json_input, dict):
                        self.logger.warn(f"recieved something from serial that was not dictionary type: {json_input}")
                        json_input = None
                    return json_input
                except json.JSONDecodeError: # did not decode json type from serial
                    self.logger.warn(f"recieved something from serial that was not json type: {serial_stream}")
        except:
            self.logger.warn(f"Problem reading from serial")
            sleep(1)
        return None

    # if Zumo is not a live, publish that serial node is alive
    # (If Zumo is alive the GUI automatically knows that serial node is alive)
    def keep_alive_publish(self):
        if not self.zumo_alive:
            msg = String()
            msg.data = "serial_interface"
            self.keep_alive_publisher_.publish(msg)
            log_msg(self.telemetry_data_logger, "published", msg)

        if (time() - self.zumo_keep_alive_time) < 1.2*KEEP_ALIVE_PERIOD:
            self.zumo_alive = True
        else:
            self.zumo_alive = False
            self.logger.warn(f"Zumo is down...")

def main(args=None):
    rclpy.init(args=args) # Initialize ROS2 libraries
    node = SerialInterfaceNode()
    try:
        publish_timer = RepeatedTimer(PUBLISH_PERIOD, node.publish) # Parse from serial and publish to topics
        keep_alive_timer = RepeatedTimer(KEEP_ALIVE_PERIOD, node.keep_alive_publish)
        rclpy.spin(node) # Make node run in background

    except KeyboardInterrupt:
        node.logger.info("Serial Interface got Keyboard Interrupt")
        pass

    finally:
        publish_timer.stop() 
        keep_alive_timer.stop()
        node.logger.info("Shutting down Serial Interface")
        node.destroy_node()

if __name__ == "__main__":
    main()