#!/usr/bin/env python3

#  Telemetry Project:
#  Renato and Michael.

# -------------IMPORTS-------------------
import rclpy
from rclpy.node import Node
import numpy as np
import subprocess
import os

from telemetry_interfaces.msg import WifiStats, Bool, Int

from telemetry_pkg.utilities import RepeatedTimer, TelemetryLogger, generate_str_timestamp, log_msg

# -------------GLOBAL PARAMETERS-----------
PUBLISH_PERIOD = 0.5 # sec

class WifiStatsNode(Node):
# ----------------INIT-------------------
    def __init__(self):
        super().__init__("wifi_stats")

        self.init_log()

        self.device_name = self.get_device_name()
        self.logger.info(f'Device is: {self.device_name}')

        self.wifi_stats_publisher_ = self.create_publisher(WifiStats, "wifi_stats", 10)

        self.logger.info("Started Wifi Stats")

    # -------------LOGGER------------
    def init_log(self):
        log_dir = os.path.join(os.path.expanduser('~'), "ros2_ws/telemetry_logs")
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        log_file_path = os.path.join(log_dir, f"wifi_stats_log_{generate_str_timestamp()}.txt")

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

    # get device by checking which device has the most TX traffic
    def get_device_name(self):
        byterate = subprocess.check_output(f'ifstat -n .1 20', shell=True, encoding="utf-8")  
        byterate = byterate.split('\n')
        devices = byterate[0].split()
        byterate = byterate[2:]
                
        data_sum = 0
        for line in byterate:
            if line == '':
                continue
            parsed_line = line.split()[1::2]
            data_sum += np.array(parsed_line).astype(np.float32)

        # return devices[data_sum.argmax()]
        return "wlx08beac1b8a82" # doesnt always get the correct device, hardcoded for now

    def get_signal_power(self):
        signal_power = -999
        try:
            signal = subprocess.check_output(f'iw dev {self.device_name} link | grep -oE "signal:(.+[0-9]+)"', shell=True, encoding="utf-8", timeout = 1)
            signal_power = int(signal[signal.find(":")+1:])
        except:
            self.logger.warn("get signal power failed")
        return signal_power

    def rx_tx_byterate(self):
        rx_tx_rate = (-1.0, -1.0)
        try:
            rx_tx_rate = subprocess.check_output(f'ifstat -n -i {self.device_name} .2 1', shell=True, encoding="utf-8", timeout = 1)
            rx_tx_rate = rx_tx_rate.split('\n')[2].split()
        except:
            self.logger.warn("rx tx byterate failed")
        return float(rx_tx_rate[0]), float(rx_tx_rate[1])

    def publish(self):
        msg = WifiStats()  
        msg.signal_power = self.get_signal_power()        
        rx_byterate, tx_byterate = self.rx_tx_byterate()
        msg.rx_rate = rx_byterate 
        msg.tx_rate = tx_byterate
        self.wifi_stats_publisher_.publish(msg)
        log_msg(self.telemetry_data_logger, "published", msg)

# ----------------MAIN-------------------
def main(args=None):
    rclpy.init(args=args) # Initialize ROS2 libraries
    node = WifiStatsNode()
    try:
        publish_timer = RepeatedTimer(PUBLISH_PERIOD, node.publish)
        rclpy.spin(node) # Make node run in background

    except KeyboardInterrupt:
        node.logger.info("Wifi Stats got Keyboard Interrupt")
        pass

    finally:
        publish_timer.stop()
        node.logger.info("Shutting down Wifi Stats")
        node.destroy_node()

if __name__ == "__main__":
    main()
