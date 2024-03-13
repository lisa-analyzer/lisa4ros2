#!/usr/bin/env python3

#  Telemetry Project:
#  Renato and Michael.

# -------------IMPORTS-------------------
import rclpy
from rclpy.node import Node
import os

from telemetry_interfaces.msg import WifiStats, Int, Bool

from telemetry_pkg.utilities import RepeatedTimer, TelemetryLogger, generate_str_timestamp, log_msg

# -------------GLOBAL PARAMETERS-----------
PUBLISH_PERIOD = 0.5 # sec
TRX_TIME = 0.2 # sec
SIGNAL_TIME = 1 # sec

class WifiGeneratorNode(Node):
# ----------------INIT-------------------
    def __init__(self):
        super().__init__("wifi_stats")
        
        self.init_log()

        self.signal_power = -20
        self.tx_rate = 0.0
        self.rx_rate = 0.0
        self.wifi_stats_publisher_ = self.create_publisher(WifiStats, "wifi_stats", 10)

        self.logger.info("Started Wifi Stats Generator")

    def trx_gen(self):
        self.tx_rate = (self.tx_rate + 10) % 350
        self.rx_rate = (self.rx_rate + 0.5) % 350
    
    def signal_gen(self):
        self.signal_power -= 1
        if self.signal_power == -70:
            self.signal_power = -30

    def init_log(self):
        log_dir = os.path.join(os.path.expanduser('~'), "ros2_ws/telemetry_logs")
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        log_file_path = os.path.join(log_dir, f"wifi_stats_generator_log_{generate_str_timestamp()}.txt")

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
        msg = WifiStats()  
        msg.signal_power = self.signal_power        
        msg.rx_rate = self.rx_rate 
        msg.tx_rate = self.tx_rate
        self.wifi_stats_publisher_.publish(msg)

        self.get_logger().debug(f"Signal Power: {msg.signal_power}")
        self.get_logger().debug(f"rx, tx: {msg.rx_rate}, {msg.tx_rate}")

        log_msg(self.telemetry_data_logger, "recieved", msg)

# ----------------MAIN-------------------
def main(args=None):
    rclpy.init(args=args) # Initialize ROS2 libraries
    node = WifiGeneratorNode()
    try:
        publish_timer = RepeatedTimer(PUBLISH_PERIOD, node.publish)
        signal_power_timer = RepeatedTimer(SIGNAL_TIME, node.signal_gen)
        trx_rate_timer = RepeatedTimer(TRX_TIME, node.trx_gen)
        rclpy.spin(node) # Make node run in background

    except KeyboardInterrupt:
        node.logger.info("Wifi Stats Generator got Keyboard Interrupt")
        pass

    finally:
        publish_timer.stop()
        signal_power_timer.stop()
        trx_rate_timer.stop()

        node.logger.info("Shutting down Wifi Stats Generator")
        node.destroy_node()

if __name__ == "__main__":
    main()
