#!/usr/bin/env python3
''' 
# ------------------------------------- #
# MARV Logger                     #
# By Viktor Lindstrom, Spring 2021      #
# Chalmers University of Technology     #
# ------------------------------------- #
# This node places markers into the rosbag when logging data, these markers can then be used 
# to aquire different logging intervals. It publishes the current logging state and a marker
# which is increased every time a new logging request is sent.
'''

# System imports
import time

# ROS Imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

# Messages
from std_msgs.msg import Bool
from std_msgs.msg import Int8

class MARV_Logger(Node):

    ##################################### INIT #############################################################
    def __init__(self):
        # Setup parameters, variables, publishers, subscribers and timers
        super().__init__('marv_logger')

        self.get_logger().info("Initializing...")

        # Global variables
        self.marker_counter = -1 # Keeps the current loggin marker, increases each time a start command is succefully sent
        self.marker_value = -1 # Marker value, is either "marker_counter" when logging or -1 when not logging
        self.is_logging = False # True if logging, False if not logging
        self.logging_marker_message_period = 0.5 # Timer interval (s)
        self.logging_state_message_period = 0.5 # Timer interval (s)

        # subscribe to start/stop logging
        self.subscription = self.create_subscription(Bool, '/marv/sys/ctrl/log_data', self.MARV_sys_logger_callback, 10)
        self.subscription

        # Publishes the logging state
        self.marv_sys_status_logging_state_publisher_ = self.create_publisher(Bool, '/marv/sys/status/logging_state', 10)
        # Publishes the current logging marker
        self.marv_sys_status_logging_marker_publisher_ = self.create_publisher(Int8, '/marv/sys/status/logging_marker', 10)

        # Allow time for subscriptions and publishes to be set up
        time.sleep(1.0)

        self.logging_marker_timer = self.create_timer(self.logging_marker_message_period, self.MARV_logging_marker_publisher_callback)
        self.logging_state_timer = self.create_timer(self.logging_state_message_period, self.MARV_logging_state_publisher_callback)

        self.get_logger().info("Running...")

    ########################################################################################################

    ##################################### CALLBACKS ########################################################

    def MARV_sys_logger_callback(self,message):
        if message.data == True and not self.is_logging: # Start logging
            if self.marker_counter < 128:
                self.is_logging = True
                self.marker_counter += 1
                self.marker_value = self.marker_counter
                self.MARV_logging_marker_publisher_callback()
                self.MARV_logging_state_publisher_callback()
                self.get_logger().info("Started logging data, marker: " + str(self.marker_value))

        elif message.data == True and self.is_logging:
            self.get_logger().info("Already logging data")

        elif message.data == False and self.is_logging: # Stop logging
            self.is_logging = False
            self.marker_value = -1
            self.MARV_logging_marker_publisher_callback()
            self.MARV_logging_state_publisher_callback()
            self.get_logger().info("Stopped logging data, marker: " + str(self.marker_value))

        elif message.data == False and not self.is_logging:
            self.get_logger().info("Already not logging data")

    def MARV_logging_marker_publisher_callback(self):
        logging_marker_message = Int8()
        logging_marker_message.data = self.marker_value
        self.marv_sys_status_logging_marker_publisher_.publish(logging_marker_message)
    
    def MARV_logging_state_publisher_callback(self):
        logging_state_message = Bool()
        logging_state_message.data = self.is_logging
        self.marv_sys_status_logging_state_publisher_.publish(logging_state_message)
        
    ########################################################################################################

    ##################################### OTHER FUNCTIONS ##################################################

    ########################################################################################################

def main(args=None):
    rclpy.init(args=args)

    marv_logger = MARV_Logger()

    rclpy.spin(marv_logger)

    marv_logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
