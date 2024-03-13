#!/usr/bin/env python3

# ------------------------------------- #
# MARV Power Management node      #
# By Viktor Lindstrom and               #
# Noel Danielsson, Spring 2021          #
# Chalmers University of Technology     #
# ------------------------------------- #

# The MARV power management node allows the ACU to be shutdown or rebooted using ROS messages.
# It also enables the on/off switch on the ACU to request a shutdown of the system.
# If GPIO cleanup fails it can be manually unexborted with: echo 268 >/sys/class/gpio/unexport

# System imports
import RPi.GPIO as GPIO
import os
import time

# ROS Imports
import rclpy
from rclpy.node import Node

# Messages
from std_msgs.msg import Int8

class MARV_Power_Management(Node):

    ##################################### INIT #############################################################
    
    def __init__(self):
        # Setup parameters, variables, publishers, subscribers and timers
        super().__init__('marv_power_management')

        self.get_logger().info("Initializing...")

        # Power options
        self.reboot = 1
        self.shutdown = 2
        
        # GPIO Pin definitions and setup
        powerButton = 22 # BCM pin 22, Board pin 15 (LCD_TE)
        self.setupButton(powerButton, self.BUT_power_callback)

        # Subscribe to various topics from sbg_driver
        self.subscription = self.create_subscription(Int8, '/marv/sys/ctrl/acu_power', self.ROS_power_callback, 10)

        # Allow time for subscriptions and publishes to be set up
        time.sleep(1.0)

        self.get_logger().info('Running...')

    ########################################################################################################

    ##################################### CALLBACKS ########################################################

    def ROS_power_callback(self,data):
        if data.data == self.shutdown:
            self.get_logger().info("Shutdown initiated via ROS...")

            # Shutdown rosbag gently
            os.system("pgrep -f 'ros2 bag' | xargs kill -SIGINT")
            
            # Wait 5 sec for rosbag to exit then execute shutdown command
            time.sleep(5.0)
            os.system("sudo shutdown now")

        elif data.data == self.reboot:
            self.get_logger().info("Reboot initiated via ROS...")

            # Shutdown rosbag gently
            os.system("pgrep -f 'ros2 bag' | xargs kill -SIGINT")

            # Wait 5 sec for rosbag to exit then execute reboot command
            time.sleep(5.0)
            os.system("sudo shutdown -r now")

        else:
            msg = "ROS: Invalid command: " + str(data.data)
            self.get_logger().error(msg)

    def BUT_power_callback(self, button):
        self.get_logger().info("Shutdown initiated via power button...")
        
        # Shutdown rosbag gently
        os.system("pgrep -f 'ros2 bag' | xargs kill -SIGINT")
        
        # Wait 5 sec for rosbag to exit then execute shutdown command
        time.sleep(5.0)
        os.system("sudo shutdown now")

    ########################################################################################################

    ##################################### OTHER FUNCTIONS ##################################################

    def setupButton(self, button, callbackFunction):
        self.get_logger().info("Setting up power button")
        GPIO.setmode(GPIO.BCM)  # BCM pin-numbering scheme from Raspberry Pi
        # set as output for short time to reset state to not pressed
        GPIO.setup(button, GPIO.OUT, initial=GPIO.HIGH)
        time.sleep(1e-6);                                   # wait for 1 us
        GPIO.setup(button, GPIO.IN)                         # set pin as an input pin
        #verify correct state
        if( GPIO.input(button) != 1):
            self.get_logger().error("Power button input not pulled high")
        # add falling edge detection
        GPIO.add_event_detect(button, GPIO.FALLING, callback=callbackFunction, bouncetime=100)

    ########################################################################################################

def main(args=None):
    rclpy.init(args=args)

    marv_power_management = MARV_Power_Management()

    # try block added to catch Ctrl+C to make sure GPIO is cleaned up
    try:
        rclpy.spin(marv_power_management)
    except KeyboardInterrupt:
        print("\n\rKeyboard interrupt")
    finally:
        marv_power_management.get_logger().info("Cleaning up GPIO")
        GPIO.cleanup()
        marv_power_management.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
