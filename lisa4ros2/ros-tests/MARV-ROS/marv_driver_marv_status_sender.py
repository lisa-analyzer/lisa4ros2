#!/usr/bin/env python3
''' 
# ------------------------------------- #
# MARV Status Sender              #
# By Viktor Lindstrom and               #
# Noel Danielsson, Spring 2021          #
# Chalmers University of Technology     #
# ------------------------------------- #
# MARV interface node between SGB_INS and the MARV ROS system.
# It repacks nessecary data, transforms geodetic to navigation coordinates, publishes status messages
# and syncs the ubuntu system time with GPS time.
#
# Script needs to have sudo privileges in order to change the system time, run "sudo su" before
# starting the ROS node
#
# Note: Before publishing any pose the node requires SBG_driver to publish data on /ekf_quat and /ekf_nav.
# The reference position must also be set.
'''

# System imports
import numpy as np
import time
import datetime
import serial

# ROS Imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

# Messages
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Vector3
from std_msgs.msg import UInt64
from std_msgs.msg import Bool
from std_msgs.msg import Int8
from std_msgs.msg import Float32
from sbg_driver.msg import SbgEkfQuat
from sbg_driver.msg import SbgEkfNav
from sbg_driver.msg import SbgUtcTime
from sbg_driver.msg import SbgStatus
from marv_msgs.msg import CmdSteering
from marv_msgs.msg import ScenarioConfig
from marv_msgs.msg import Notification
from marv_msgs.msg import Status
from marv_msgs.msg import Log1NCU
from marv_msgs.msg import Log1TCU

class MARV_Status_Sender(Node):

    ##################################### INIT #############################################################
    def __init__(self):
        # Setup parameters, variables, publishers, subscribers and timers
        super().__init__('marv_status_sender')

        self.get_logger().info("Initializing...")

        # Configuration parameters
        self.status_message_period = 1.0 # seconds update time for sending status message

        # Other global variables
        self.current_time = 0 # Keep current system time
        self.message_timeout = 3 # Timeout for when a message has not been recived (s)

        # Callback variables
        self.efk_status = -1 # -1 = not set from sbg_driver, 0 = UNINITIALIZED, 1 = VERTICAL_GYRO, 2 = AHRS, 3 = NAV_VELOCITY, 4 = NAV_POSITION
        self.sys_unix_time_with_offset = 0 # Keep unix time from system
        self.unix_time_with_offset = 0 # Keep unix time from GPS sync
        self.reference_position_geo = np.array(((0.0),(0.0),(0.0))) # Reference position for converting geodetic coordinates (lat,long,alt) to navigation coordinates (x,y,z), z not used
        self.current_pos_nav = np.array(((0.0),(0.0),(0.0))) # Current position in navigation coordinates
        self.current_pos_geo = np.array(((0.0),(0.0),(0.0))) # Current position in geodetic coordinates
        self.current_orientation_rpy = np.array(((0.0),(0.0),(0.0))) # Current orientation in rpy
        self.current_velocity_magnitude = 0 # Current velocity magnitude, using the velocity along x,y axis
        self.current_throttle_aps = 0
        self.current_throttle_rps = 0
        self.current_steering_angle = 0
        self.current_mode = 0
        self.status = [False,False,False] # SbgEkfStatus, heading, velocity, position
        self.scenario_state = 3

        # Scenario states
        self.handler_state = {
            0: "WA",
            1: "EX",
            2: "FA",      
            3: "ST",     
            4: "FI",     
            5: "RE"     
        }

        # Last time message recieved variables
        self.efk_status_time = -1
        self.sys_unix_time_with_offset_time = -1
        self.unix_time_with_offset_time = -1
        self.reference_position_geo_time = -1
        self.current_pos_nav_time = -1
        self.current_pos_geo_time = -1
        self.current_velocity_magnitude_time = -1
        self.current_throttle_time = -1
        self.current_steering_angle_time = -1
        self.current_mode_time = -1
        self.status_time = -1
        self.scenario_state_time = -1

        # Subscribe to marv sbg pose
        self.subscription = self.create_subscription(PoseWithCovariance, '/marv/nav/sbg_pose', self.MARV_sbg_pose_callback, 10)
        # Subscribe to sbg ekf filter status
        self.subscription = self.create_subscription(Int8, '/marv/nav/sbg_ekf_status', self.MARV_sbg_ekf_status_callback, 10)
        # Subscribe to sbg time (GPS)
        self.subscription = self.create_subscription(UInt64, '/marv/nav/sbg_time', self.MARV_sbg_time_callback, 10)
        # Subscribe to the system time
        self.subscription = self.create_subscription(UInt64, '/marv/nav/sys_time', self.MARV_sys_time_callback, 10)
        # Subscribe to reference position
        self.subscription = self.create_subscription(Vector3, '/marv/nav/sbg_ref_pos', self.MARV_sbg_ref_pos_callback, 10)
        # Subscribe to sbg pos geodetic
        self.subscription = self.create_subscription(Vector3, '/marv/nav/sbg_current_pos', self.MARV_sbg_current_pos_callback, 10)
        # Subscribe to sbg velocity magnitude
        self.subscription = self.create_subscription(Float32, '/marv/nav/sbg_velocity_magnitude', self.MARV_sbg_current_vel_mag_callback, 10)
        # Subscribe to marv steering angle log
        self.subscription = self.create_subscription(Log1NCU, '/marv/sys/log/log1_ncu', self.MARV_sys_log_1NCU_callback, 10)
        # Subscribe to marv throttle signal
        self.subscription = self.create_subscription(Log1TCU, '/marv/sys/log/log1_tcu', self.MARV_sys_log_1TCU_callback, 10)
        # subscribe to marv mode
        self.subscription = self.create_subscription(Int8, '/marv/sys/status/mode', self.MARV_sys_status_mode_callback, 10)
        # subscribe to SbgEkfNav
        self.subscription = self.create_subscription(SbgEkfNav, '/sbg/ekf_nav', self.SbgEkfNav_callback, 10)
        # Subscribe to scenario state
        self.subscription = self.create_subscription(Int8, '/marv/sys/ctrl/scenario_state', self.MARV_sys_ctrl_scenario_state_callback, 10)
        # Subscribe to steering commands from scenario
        self.subscription

        #init of serial com:
        self.ser = self.initSerial()
        
        #check if serial port is open
        if(self.ser.isOpen()):
            self.get_logger().info("Serial port opened")

        # Allow time for subscriptions and publishes to be set up
        time.sleep(1.0)

        # Setup timers for periodic callbacks
        self.status_message_timer = self.create_timer(self.status_message_period, self.MARV_sbg_status_sender_callback)

        self.get_logger().info("Running...")

    ########################################################################################################

    ##################################### SERIAL MESSAGE CALLBACK ##########################################

    def MARV_sbg_status_sender_callback(self):
        #print("Sending serial data: " + datetime.datetime.now().strftime('%H:%M:%S'))
        self.current_time = time.time()

        serial_message = "\033c \n\r"

        serial_message += "TIME\n\r"
        serial_message += self.check_last_time(self.sys_unix_time_with_offset_time) + "SYS_time: " + datetime.datetime.fromtimestamp(self.sys_unix_time_with_offset).strftime('%Y-%m-%d %H:%M:%S') + "\n\r"
        serial_message += self.check_last_time(self.unix_time_with_offset_time) + "GPS_time: " + datetime.datetime.fromtimestamp(self.unix_time_with_offset).strftime('%Y-%m-%d %H:%M:%S') + "\n\r"

        serial_message += "POSITION_STATUS\n\r"
        serial_message += self.check_last_time(self.efk_status_time) + "EKF_status: " + str(self.efk_status) + "\n\r"
        serial_message += self.check_last_time(self.current_pos_nav_time) + "POS_nav: " + str(self.current_pos_nav) + "\n\r"
        serial_message += self.check_last_time(self.current_pos_nav_time) + "ORI_rpy: " + str(self.current_orientation_rpy) + "\n\r"
        serial_message += self.check_last_time(self.current_pos_geo_time) + "POS_geo: " + str(self.current_pos_geo) + "\n\r"
        serial_message += self.check_last_time(self.reference_position_geo_time) + "POS_ref: " + str(self.reference_position_geo) + "\n\r"
        serial_message += self.check_last_time(self.current_velocity_magnitude_time) + "VEL_mag: " + str(self.current_velocity_magnitude) + "\n\r"

        serial_message += "WAVERUNNER_SYS\n\r"
        serial_message += self.check_last_time(self.current_throttle_time) + "THR_aps: " + str(self.current_throttle_aps) + "\n\r"
        serial_message += self.check_last_time(self.current_throttle_time) + "THR_rps: " + str(self.current_throttle_rps) + "\n\r"
        serial_message += self.check_last_time(self.current_steering_angle_time) + "STE_ang: " + str(self.current_steering_angle) + "\n\r"
        if self.current_mode == 0:
            serial_message += self.check_last_time(self.current_mode_time) + "MODE: man \n\r"
        else:
            serial_message += self.check_last_time(self.current_mode_time) + "MODE: ext \n\r"

        serial_message += "SCENARIO_STATUS\n\r"
        serial_message += self.check_last_time(self.scenario_state_time) + "SCN_state: " + self.handler_state[self.scenario_state] + "\n\r"

        serial_message += "SBG_STATUS\n\r"
        serial_message += self.check_last_time(self.status_time) + "HED_val: " + str(self.status[0]) + "\n\r"
        serial_message += self.check_last_time(self.status_time) + "VEL_val: " + str(self.status[1]) + "\n\r"
        serial_message += self.check_last_time(self.status_time) + "POS_val: " + str(self.status[2]) + "\n\r"

        self.sendSerialCommand(serial_message)

    ########################################################################################################

    ##################################### CALLBACKS ########################################################

    def MARV_sbg_pose_callback(self,data):
        self.current_pos_nav_time = time.time()
        self.current_pos_nav[0] = data.pose.position.x
        self.current_pos_nav[1] = data.pose.position.y
        self.current_pos_nav[2] = data.pose.position.z
        self.current_orientation_rpy = self.quaternion_to_euler_deg(data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w)

    def MARV_sbg_ekf_status_callback(self,data):
        self.efk_status_time = time.time()
        self.efk_status = data.data

    def MARV_sys_time_callback(self,data):
        self.sys_unix_time_with_offset_time = time.time()
        self.sys_unix_time_with_offset = data.data

    def MARV_sbg_time_callback(self,data):
        self.unix_time_with_offset_time = time.time()
        self.unix_time_with_offset = data.data

    def MARV_sbg_ref_pos_callback(self,data):
        self.reference_position_geo_time = time.time()
        self.reference_position_geo = np.array(((data.x),(data.y),(data.z)))

    def MARV_sbg_current_pos_callback(self,data):
        self.current_pos_geo_time = time.time()
        self.current_pos_geo = np.array(((data.x),(data.y),(data.z)))

    def MARV_sbg_current_vel_mag_callback(self,data):
        self.current_velocity_magnitude_time = time.time()
        self.current_velocity_magnitude = data.data

    def MARV_sys_status_mode_callback(self,data):
        self.current_mode_time = time.time()
        self.current_mode = data.data

    def MARV_sys_log_1NCU_callback(self,data):
        self.current_steering_angle_time = time.time()
        self.current_steering_angle = data.angle

    def MARV_sys_log_1TCU_callback(self,data):
        self.current_throttle_time = time.time()
        self.current_throttle_aps = data.aps_out
        self.current_throttle_rps = data.rps_out

    def SbgEkfNav_callback(self,data):
        self.status_time = time.time()
        self.status = [data.status.heading_valid, data.status.velocity_valid, data.status.position_valid]

    def MARV_sys_ctrl_scenario_state_callback(self,data):
        self.scenario_state_time = time.time()
        self.scenario_state = data.data

    ########################################################################################################

    ##################################### OTHER FUNCTIONS ##################################################

    def sendSerialCommand(self,message):
        self.ser.write(bytes(message, 'cp437'))

    def initSerial(self):
        #TODO: update to make these configurable arguments
        #TODO: add exception handling when the port fails to be configured
        # Serial port is instantly opened upon creation (https://pyserial.readthedocs.io/en/latest/pyserial_api.html)
        ser = serial.Serial(
        port='/dev/ttyTHS0',
        baudrate=115200,
        timeout = 3)
        return ser

    def closeSerial(self):
        self.ser.close()
        self.get_logger.info("Closing serial port")

    def check_last_time(self, last_time):
        serial_message = ""
        if self.current_time - last_time > self.message_timeout:
            serial_message += "s " # "s" for not synced, we have not recived any message within the timeout
        else:
            serial_message += "  "
        return serial_message
        
    #Convert quaternions to roll pitch yaw angles in deg
    def quaternion_to_euler_deg(self, x, y, z, w):
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x**2 + y**2)
        roll = np.rad2deg(np.arctan2(sinr_cosp, cosr_cosp))

        sinp = 2 * (w * y - z * x)
        if abs(sinp) >=1:
            pitch = np.rad2deg(np.sign(sinp) * np.pi/2)
        else:
            pitch = np.rad2deg(np.arcsin(sinp))

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y**2 + z**2)
        yaw = np.rad2deg(np.arctan2(siny_cosp, cosy_cosp))

        return np.array(((roll),(pitch),yaw))

    ########################################################################################################

def main(args=None):
    rclpy.init(args=args)

    marv_status_sender = MARV_Status_Sender()

    rclpy.spin(marv_status_sender)

    marv_status_sender.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
