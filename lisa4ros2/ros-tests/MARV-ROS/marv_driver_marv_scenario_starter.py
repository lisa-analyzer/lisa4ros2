#!/usr/bin/env python3
''' 
# ------------------------------------- #
# MARV Scenario Starter           #
# By Viktor Lindstrom, Spring 2021      #
# Chalmers University of Technology     #
# ------------------------------------- #
# This node acts as a simulator for the OCU unit. It provides a way
# to start scenarios without having access to the physical system.
'''

# System imports
import numpy as np
import time

# ROS Imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

# Messages
from std_msgs.msg import UInt8
from std_msgs.msg import Int8
from std_msgs.msg import Bool
from marv_msgs.msg import Notification
from marv_msgs.msg import Status

class MARV_Scenario_Starter(Node):

    ##################################### INIT #############################################################
    def __init__(self):
        # Setup parameters, variables, publishers, subscribers and timers
        super().__init__('marv_scenario_starter')

        print("Starting...")

        # Variables
        self.notification_message = ""
        self.scenario_start_timeout_update_period = 5.0
        self.scenario_switch_to_external_mode_period = 2.0
        self.user_message_sent = False
        self.wait_time_started = False
        self.executing_message_sent = False
        self.reset_next = False

        # Scenario states
        self.handler_state = {
            "WAITING":      0,
            "EXECUTING":    1,
            "FAULT":        2,
            "STOPPED":      3,
            "FINISHED":     4,
            "REQ_SENT":     5
        }

        self.last_state = self.handler_state["STOPPED"]

        # Publishes the 12V auto state
        self.marv_sys_status_12V_Auto_publisher_ = self.create_publisher(Bool, '/marv/sys/status/state_12V_auto', 10)
        # Publishes start requests
        self.marv_sys_status_start_scenario_publisher_ = self.create_publisher(UInt8, '/marv/sys/status/start_scenario', 10)

        # Heartbeat TCU
        self.marv_sys_status_heartbeat_tcu_publisher_ = self.create_publisher(Status, '/marv/sys/status/heartbeat_tcu', 10)
        # Heartbeat PDU
        self.marv_sys_status_heartbeat_pdu_publisher_ = self.create_publisher(Status, '/marv/sys/status/heartbeat_pdu', 10)
        # Heartbeat NCU
        self.marv_sys_status_heartbeat_ncu_publisher_ = self.create_publisher(Status, '/marv/sys/status/heartbeat_ncu', 10)

        # Subscribe to notification messages
        self.subscription = self.create_subscription(Notification, '/marv/sys/ctrl/formatted_notification_message', self.notification_message_callback, 20)
        # Subscribe to scenario handler state
        self.subscription = self.create_subscription(Int8, '/marv/sys/ctrl/scenario_state', self.scenario_handler_state_callback, 10)

        # Setup timers
        self.scenario_start_timeout_timer = self.create_timer(self.scenario_start_timeout_update_period,self.scenario_start_timeout_callback)
        self.scenario_start_timeout_timer.cancel()
        self.scenario_switch_to_external_mode_timer = self.create_timer(self.scenario_switch_to_external_mode_period,self.scenario_switch_to_external_mode_callback)
        self.scenario_switch_to_external_mode_timer.cancel()

        # Allow time for subscriptions and publishes to be set up
        time.sleep(1.0)
        print("Initialized, waiting for scenario handler...")

    ########################################################################################################

    ##################################### CALLBACKS ########################################################

    def notification_message_callback(self,ros_msg):
        if ros_msg.location == 0:
            self.notification_message = ""
        for i in range(0,7):
            self.notification_message += chr(ros_msg.data[i])
        if ros_msg.location == 9:
            print(str(self.notification_message))

    def scenario_start_timeout_callback(self):
        print("Scenario did not start")
        self.scenario_start_timeout_timer.cancel()
        self.scenario_switch_to_external_mode_timer.cancel()
        self.reset_next = True

    def scenario_switch_to_external_mode_callback(self):
        mode_message = Bool()
        mode_message.data = True
        self.marv_sys_status_12V_Auto_publisher_.publish(mode_message)
        print("Switching to external mode")
        self.scenario_switch_to_external_mode_timer.cancel()

    def scenario_handler_state_callback(self,ros_msg):
        if self.last_state != ros_msg.data:
            self.last_state = ros_msg.data

        if ros_msg.data == self.handler_state["WAITING"] and not self.wait_time_started:
            self.wait_time_started = True
            print("The handler is waiting to start, switching to external mode in 2 seconds")
            self.scenario_switch_to_external_mode_timer.reset()
            self.scenario_start_timeout_timer.cancel()

        elif ros_msg.data == self.handler_state["STOPPED"] and not self.user_message_sent:
            self.user_message_sent = True

            chosen_scenario = input("Enter the scenario id (from scenarios.yaml, an integer) to start: ")
            self.publish_heartbeat()
            time.sleep(0.5)
            start_message = UInt8()
            start_message.data = int(chosen_scenario)
            self.marv_sys_status_start_scenario_publisher_.publish(start_message)
            print("Sent a start request to scenario: " + str(chosen_scenario))
            
            self.scenario_start_timeout_timer.reset()
        elif ros_msg.data == self.handler_state["EXECUTING"] and not self.executing_message_sent:
            self.executing_message_sent = True
            print("The scenario is executing")
            input("Press enter to deactivate external mode")
            mode_message = Bool()
            mode_message.data = False
            self.marv_sys_status_12V_Auto_publisher_.publish(mode_message)
            time.sleep(1.0)
            print("External mode deactivated")
            self.reset_next = True

        if self.reset_next:
            self.reset_next = False
            self.user_message_sent = False
            self.wait_time_started = False
            self.executing_message_sent = False
            self.scenario_start_timeout_timer.cancel()
            self.scenario_switch_to_external_mode_timer.cancel()
            print("Scenario handler switched to stopped state, resetting.")

        self.publish_heartbeat()
        
    def publish_heartbeat(self):
        status_message = Status()
        status_message.status = 1
        self.marv_sys_status_heartbeat_tcu_publisher_.publish(status_message)
        self.marv_sys_status_heartbeat_ncu_publisher_.publish(status_message)
        self.marv_sys_status_heartbeat_pdu_publisher_.publish(status_message)

    ########################################################################################################

    ##################################### OTHER FUNCTIONS ##################################################

    ########################################################################################################

def main(args=None):
    rclpy.init(args=args)

    marv_scenario_starter = MARV_Scenario_Starter()

    rclpy.spin(marv_scenario_starter)

    marv_scenario_starter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
