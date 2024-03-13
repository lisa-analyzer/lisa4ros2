#!/usr/bin/env python3
''' 
# ------------------------------------- #
# MARV ACU Heartbeat              #
# By Viktor Lindstrom, Spring 2021      #
# Chalmers University of Technology     #
# ------------------------------------- #
# This node publishes a heartbeat for the OCU to the DBW system. The status of the
# heartbeat can be set by sending a ROS message to this node.
'''

# System imports
import time

# ROS Imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

# Messages
from std_msgs.msg import Int8
from marv_msgs.msg import Status

class MARV_Heartbeat_ACU(Node):

    ##################################### INIT #############################################################
    def __init__(self):
        # Setup parameters, variables, publishers, subscribers and timers
        super().__init__('marv_heartbeat_acu')

        self.get_logger().info("Initializing...")

        # Configuration parameters 
        self.heartbeat_message_period = 0.1 # Timer interval (s)

        # Global variables
        
        # Heartbeat states
        self.state = {
            "ERROR":  0,
            "OKAY":   1,
        }

        # Heartbeat state variable
        self.heartbeat_state =  self.state["OKAY"]
        self.heartbeat_error_code = 0 # 0 no error code set

        # subscribe to start/stop logging
        self.subscription = self.create_subscription(Status, '/marv/sys/ctrl/heartbeat_acu_state', self.update_acu_heatbeat_state_callback, 10)
        self.subscription

        # Publishes the acu heartbeat state
        self.marv_sys_acu_heartbeat_publisher_ = self.create_publisher(Status, '/marv/sys/status/heartbeat_acu', 10)

        # Allow time for subscriptions and publishes to be set up
        time.sleep(1.0)

        self.acu_heartbeat_timer = self.create_timer(self.heartbeat_message_period, self.acu_heartbeat_timer_callback)
    
        self.get_logger().info("Running...")

    ########################################################################################################

    ##################################### CALLBACKS ########################################################

    def acu_heartbeat_timer_callback(self):
        heartbeat_message = Status()
        heartbeat_message.status = int(self.heartbeat_state)
        heartbeat_message.error_code = int(self.heartbeat_error_code)
        self.marv_sys_acu_heartbeat_publisher_.publish(heartbeat_message)

    def update_acu_heatbeat_state_callback(self,ros_msg):
        if ros_msg.data == self.state["OKAY"] or ros_msg.data == self.state["ERROR"]:
            self.heartbeat_state = ros_msg.status
            self.heartbeat_error_code = ros_msg.error_code
        
    ########################################################################################################

def main(args=None):
    rclpy.init(args=args)

    marv_heartbeat_acu = MARV_Heartbeat_ACU()

    rclpy.spin(marv_heartbeat_acu)

    marv_heartbeat_acu.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
