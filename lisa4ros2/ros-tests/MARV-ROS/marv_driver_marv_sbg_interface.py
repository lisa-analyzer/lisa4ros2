#!/usr/bin/env python3
''' 
# ------------------------------------- #
# MARV SBG Interface node         #
# By Viktor Lindstrom, Spring 2021      #
# Chalmers University of Technology     #
# ------------------------------------- #
# MARV interface node between SGB_INS and the MARV ROS system.
# It repacks nessecary data, transforms geodetic to navigation coordinates, publishes status messages
# and syncs the ubuntu system time with GPS time.
#
# Script needs to have sudo privileges in order to change the system time, run "sudo su" before
# starting the ROS node, or disable the requirment of password for sudo commands
#
# Note: Before publishing any pose the node requires SBG_driver to publish data on /ekf_quat and /ekf_nav.
# The reference position must also be set.
'''

# System imports
import numpy as np
import time
import datetime
import os

# Custom libraries
from timer import Timer

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
from sbg_driver.msg import SbgEkfEuler
from sbg_driver.msg import SbgEkfNav
from sbg_driver.msg import SbgUtcTime
from sbg_driver.msg import SbgStatus
from sbg_driver.msg import SbgImuData

class MARV_SBG_Interface(Node):

    ##################################### INIT #############################################################
    def __init__(self):
        # Setup parameters, variables, publishers, subscribers and timers
        super().__init__('marv_sbg_interface')

        self.get_logger().info("Initializing...")

        # Configuration parameters
        self.state_message_period = 1.0 # seconds update time for publishing "marv_sbg_ref_pos_state", "marv_sbg_time_synced_state" and "marv_sbg_gps_pos_reliable_state"
        self.user_message_period = 0.1 # seconds update time for showing user info in cmd window
        self.sys_time_message_period = 0.2 # seconds update time for publishing the marv system time
        self.UTC_time_zone = 2 # Time zoned used
        self.user_output = False # If true, then user information will be shown in cmd monitor

        # Global variables
        #self.ref_pos_set = False # Keeps track if reference position is set
        self.ref_pos_set = True # Keeps track if reference position is set
        self.time_synced = False # Keeps track if time is correctly synced
        self.efk_status = -1 # -1 = not set from sbg_driver, 0 = UNINITIALIZED, 1 = VERTICAL_GYRO, 2 = AHRS, 3 = NAV_VELOCITY, 4 = NAV_POSITION
        self.gps_pos_reliable = False # If GPS data is reliable < 10m error
        self.connection_with_sbg_driver = False # If this node has recieved anything from the SBG_driver node
        self.unix_time_with_offset = 0 # Keep unix time from GPS sync

        self.accelerometer_reading = np.array(((0.0),(0.0),(0.0))) # Acceleration from IMU, X, Y, Z axis m/s2 
        self.gyroscope_reading = np.array(((0.0),(0.0),(0.0))) # Angular velocity from IMU, X, Y, Z axis rad/s 

        #self.reference_position_geo = np.array(((0.0),(0.0),(0.0))) # Reference position for converting geodetic coordinates (lat,long,alt) to navigation coordinates (x,y,z), z not used
        self.reference_position_geo = np.array(((57.725028),(11.649762),(0.0)))
        self.current_pos_nav = np.array(((0.0),(0.0),(0.0))) # Current position in navigation coordinates, NED
        self.current_pos_geo = np.array(((0.0),(0.0),(0.0))) # Current position in geodetic coordinates, WGS84
        self.current_pos_geo_accuracy = np.array(((0.0),(0.0),(0.0))) # Current position in geodetic coordinates accuracy for lat,long,alt (m) (1 sigma)
        self.position_updated = False # Is set to true when updated data is recieved from sbg unit
        self.current_velocity_magnitude = 0 # Current velocity magnitude, using the velocity along x,y axis (BODY)
        self.current_velocity_nav = np.array(((0.0),(0.0),(0.0))) # Current velocity along x,y,x (NED)

        self.current_orientation = np.array(((0.0),(0.0),(0.0),(0.0))) # Current orientation in quaternions
        self.current_orientation_accuracy = np.array(((0.0),(0.0),(0.0))) # Current orientation angle accuracy for roll,pitch,yaw (rad) (1 sigma)
        self.orientation_updated = False # Is set to true when updated data is recieved from sbg unit

        # Custom timers
        self.rateLimit_ocu_status = 0.0005 # Seconds between ocu ins update messages
        self.rateLimit_ocu_status_timer = Timer()
        self.rateLimit_ocu_status_timer.start()

        # Subscribe to various topics from sbg_driver
        self.subscription = self.create_subscription(SbgEkfQuat, '/sbg/ekf_quat', self.SbgEkfQuat_callback, 10)
        self.subscription = self.create_subscription(SbgEkfEuler, '/sbg/ekf_euler', self.SbgEkfEuler_callback, 10)
        self.subscription = self.create_subscription(SbgEkfNav, '/sbg/ekf_nav', self.SbgEkfNav_callback, 10)
        self.subscription = self.create_subscription(SbgUtcTime, '/sbg/utc_time', self.SbgUtcTime_callback, 10)
        self.subscription = self.create_subscription(SbgImuData, '/sbg/imu_data', self.SbgImuData_callback, 10)
        # Subscribe to reference position, for converting geodetic coordinates to navigation coordinates
        self.subscription = self.create_subscription(Vector3, '/marv/nav/sbg_ref_pos', self.MARV_ref_pos_callback, 10)
        self.subscription

        # Publishes the pose with covariance, position in NED frame
        self.marv_sbg_pose_publisher_ = self.create_publisher(PoseWithCovariance, '/marv/nav/sbg_pose', 10)
        # Publishes the system time
        self.marv_sys_time_publisher_ = self.create_publisher(UInt64, '/marv/nav/sys_time', 10)
        # Publishes the current synced GPS time
        self.marv_sbg_time_publisher_ = self.create_publisher(UInt64, '/marv/nav/sbg_time', 10)
        # Publishes the current state of the node, reference position (false = not set, true = set => node will output pose)
        self.marv_sbg_ref_pos_state_publisher_ = self.create_publisher(Bool, '/marv/nav/sbg_ref_pos_state', 10)
        # Publishes the current state of the node, time synced (false = not synced, true = synced, system time is set, time is also published)
        self.marv_sbg_time_synced_state_publisher_ = self.create_publisher(Bool, '/marv/nav/sbg_time_synced_state', 10)
        # Publishes the gps_pos_reliable state, (false = not reliable, true = reliable <10m)
        self.marv_sbg_gps_pos_reliable_state_publisher_ = self.create_publisher(Bool, '/marv/nav/sbg_gps_pos_reliable_state', 10)
        # Publishes the ekf status, (-1 = not set from sbg_driver, 0 = UNINITIALIZED, 1 = VERTICAL_GYRO, 2 = AHRS, 3 = NAV_VELOCITY, 4 = NAV_POSITION)
        self.marv_sbg_ekf_status_publisher_ = self.create_publisher(Int8, '/marv/nav/sbg_ekf_status', 10)
        # Publishes the current position in latitude longitude altitude
        self.marv_sbg_current_pos_publisher_ = self.create_publisher(Vector3, '/marv/nav/sbg_current_pos', 10)
        # Publishes the current velocity magnitude
        self.marv_sbg_velocity_magnitude_publisher_ = self.create_publisher(Float32, '/marv/nav/sbg_velocity_magnitude', 10)
        # Publishes the current velocity
        self.marv_sbg_velocity_publisher_ = self.create_publisher(Vector3, '/marv/nav/sbg_velocity', 10)
        # Publishes the current acceleration
        self.marv_sbg_acceleration_publisher_ = self.create_publisher(Vector3, '/marv/nav/sbg_acceleration', 10)
        # Publishes the current angular velocity
        self.marv_sbg_angular_velocity_publisher_ = self.create_publisher(Vector3, '/marv/nav/sbg_angular_velocity', 10)
        # Publishes the ins status for heading, velocity and position
        self.marv_sbg_ins_status_publisher_ = self.create_publisher(Int8, '/marv/nav/sbg_ins_status', 10)

        # Allow time for subscriptions and publishes to be set up
        time.sleep(1.0)

        # Setup timers for periodic callbacks
        self.state_message_timer = self.create_timer(self.state_message_period, self.MARV_sbg_state_publisher_callback)
        self.user_info_timer = self.create_timer(self.user_message_period, self.user_message_callback)
        self.sys_time_message_timer = self.create_timer(self.sys_time_message_period, self.sys_time_message_callback)

    ########################################################################################################

    ##################################### CALLBACKS ########################################################
    # Listens to the SBG orientation data message, updates variables and publishes 
    # new pose if both pos and orientation is updated with new data
    def SbgEkfQuat_callback(self, data):
        self.connection_with_sbg_driver = True

        self.current_orientation = np.array(((data.quaternion.x),(data.quaternion.y),(data.quaternion.z),(data.quaternion.w)))
        self.current_orientation_accuracy = np.array(((data.accuracy.x),(data.accuracy.y),(data.accuracy.z)))
        self.orientation_updated = True

        #Publish new pose if new data has been recived from both SbgEkfQuat_callback and SbgEkfNav_callback
        if self.ref_pos_set and (self.efk_status == 3 or self.efk_status == 4):
            self.publish_pose()

    # Listnes to the SBG navigation geodetic position message, updates variables and publishes 
    # new pose if both pos and orientation is updated with new data
    def SbgEkfNav_callback(self, data):
        self.connection_with_sbg_driver = True

        self.current_pos_geo = np.array(((data.position.x),(data.position.y),(data.position.z)))
        self.current_pos_geo_accuracy = np.array(((data.position_accuracy.x),(data.position_accuracy.y),(data.position_accuracy.z)))
        self.position_updated = True

        # Update gps_pos_reliable variable (if gps position is valid <10m)
        self.gps_pos_reliable = data.status.position_valid

        if self.rateLimit_ocu_status_timer.elapsed() > self.rateLimit_ocu_status:
            self.rateLimit_ocu_status_timer.reset()

            # Update the ekf filter status variable (SbgEkfStatus.solution_mode)
            self.efk_status = data.status.solution_mode
            ekf_status_message = Int8()
            ekf_status_message.data = self.efk_status
            self.marv_sbg_ekf_status_publisher_.publish(ekf_status_message)
            self.marv_sbg_ekf_status_publisher_.publish(ekf_status_message)
            self.marv_sbg_ekf_status_publisher_.publish(ekf_status_message)
            self.marv_sbg_ekf_status_publisher_.publish(ekf_status_message)
            self.marv_sbg_ekf_status_publisher_.publish(ekf_status_message)
            self.marv_sbg_ekf_status_publisher_.publish(ekf_status_message)
            self.marv_sbg_ekf_status_publisher_.publish(ekf_status_message)
            self.marv_sbg_ekf_status_publisher_.publish(ekf_status_message)

            # Publish the SbgEkfNav status in an integer, for displaying on the OCU
            ins_status = 0
            if data.status.heading_valid == True:
                ins_status += 100
            if data.status.velocity_valid == True:
                ins_status += 10
            if data.status.position_valid == True:
                ins_status += 1
            ins_status_message = Int8()
            ins_status_message.data = ins_status
            self.marv_sbg_ins_status_publisher_.publish(ins_status_message)

        # Update current velocity in NED
        self.current_velocity_nav = np.array(((data.velocity.x),(data.velocity.y),(data.velocity.z)))

        #Publish new pose if new data has been recived from both SbgEkfQuat_callback and SbgEkfNav_callback
        if self.ref_pos_set and (self.efk_status == 3 or self.efk_status == 4):
            self.current_pos_nav = self.ENU_to_NED(self.WGS84_to_ENU(self.reference_position_geo, self.current_pos_geo)) # Transform geodetic WSG84 coordinates to local navigation coordinates NED
            self.publish_pose()
            #self.get_logger().info("POS: " + str(self.current_pos_nav))

        # Calculate and publish the marv velocity magnitude
        self.current_velocity_magnitude = np.sqrt(np.power(data.velocity.x,2) + np.power(data.velocity.y,2))
        current_velocity_magnitude_message = Float32()
        current_velocity_magnitude_message.data = self.current_velocity_magnitude
        self.marv_sbg_velocity_magnitude_publisher_.publish(current_velocity_magnitude_message)

    def SbgEkfEuler_callback(self, data):
        self.connection_with_sbg_driver = True

        roll = data.angle.x
        pitch = data.angle.y
        yaw = data.angle.z

        self.current_orientation = self.euler_to_quaternion(roll, pitch, yaw)
        self.current_orientation_accuracy = np.array(((data.accuracy.x),(data.accuracy.y),(data.accuracy.z)))
        self.orientation_updated = True

        #Publish new pose if new data has been recived from both SbgEkfQuat_callback and SbgEkfNav_callback
        if self.ref_pos_set and (self.efk_status == 3 or self.efk_status == 4):
            self.publish_pose()

    # Listens to the synced GPS time, also updates the ubuntu system time the first run when the GPS time is synced
    def SbgUtcTime_callback(self, data):
        self.connection_with_sbg_driver = True

        # Read time
        year = data.year
        month = data.month
        day = data.day
        hour = data.hour
        minute = data.min
        second = data.sec
        microsecond = int(data.nanosec/1000)

        # Convert to unix timestamp
        unix_timezone_offset = 3600 * self.UTC_time_zone
        unix_time = datetime.datetime(year,month,day,hour,minute,second,microsecond).timestamp()
        self.unix_time_with_offset = unix_time + unix_timezone_offset

        #Get UTC clock sync state from sbg
        #utc_time_synced = data.clock_status.clock_utc_sync

        # Run when the UTC clock is synced
        if self.time_synced:
            unix_time_with_offset_message = UInt64()
            unix_time_with_offset_message.data = int(self.unix_time_with_offset)
            self.marv_sbg_time_publisher_.publish(unix_time_with_offset_message)
        
        # Run once when the UTC clock is synced
        elif data.clock_status.clock_utc_sync and not self.time_synced: 
            self.time_synced = True
            self.update_time(self.unix_time_with_offset) #Requires rosnode to be run after "sudo su"
            self.get_logger().info("System time synced with GPS")

    # Listens to SBG imu data
    def SbgImuData_callback(self, data):
        
        # Publishes the current acceleration
        self.accelerometer_reading = np.array(((data.accel.x),(data.accel.y),(data.accel.z)))
        acceleration_message = Vector3()
        acceleration_message.x = float(self.accelerometer_reading[0])
        acceleration_message.y = float(self.accelerometer_reading[1])
        acceleration_message.z = float(self.accelerometer_reading[2])
        self.marv_sbg_acceleration_publisher_.publish(acceleration_message)

        # Publishes the current angular velocity
        self.gyroscope_reading = np.array(((data.gyro.x),(data.gyro.y),(data.gyro.z)))
        angular_velocity_message = Vector3()
        angular_velocity_message.x = float(self.gyroscope_reading[0])
        angular_velocity_message.y = float(self.gyroscope_reading[1])
        angular_velocity_message.z = float(self.gyroscope_reading[2])
        self.marv_sbg_angular_velocity_publisher_.publish(angular_velocity_message)

    # Updates the reference position when provided from an external node
    def MARV_ref_pos_callback(self, data):
        self.reference_position_geo = np.array(((data.x),(data.y),(data.z)))
        self.ref_pos_set = True
        self.get_logger().info("Reference pos set to " + str(self.reference_position_geo))

    # Publishes the ref_pos_set and time_synced states
    def MARV_sbg_state_publisher_callback(self):
        # Messages for keeping the two states of the node
        state_ref_pos_set_message = Bool()
        state_time_synced_message = Bool()
        state_gps_pos_reliable_message = Bool()

        state_ref_pos_set_message.data = self.ref_pos_set
        state_time_synced_message.data = self.time_synced
        state_gps_pos_reliable_message.data = self.gps_pos_reliable

        self.marv_sbg_ref_pos_state_publisher_.publish(state_ref_pos_set_message)
        self.marv_sbg_time_synced_state_publisher_.publish(state_time_synced_message)
        self.marv_sbg_gps_pos_reliable_state_publisher_.publish(state_gps_pos_reliable_message)
        
    # Publishes the system time
    def sys_time_message_callback(self):
        current_system_time_message = UInt64()
        current_system_time_message.data = int(time.time())
        self.marv_sys_time_publisher_.publish(current_system_time_message)

    # Show user information in command window
    def user_message_callback(self):
        if self.user_output:
            print("\033c")

            print("################# USER INFO - MARV SBG Interface #################")

            print(" ")

            if self.connection_with_sbg_driver:
                print(" SBG_driver connection:       Connected")
            else:
                print(" SBG_driver connection:       Not connected")

            print(" EKF status:                  {0:d}".format(self.efk_status))
            
            if self.gps_pos_reliable:
                print(" GPS pos reliable:            True")
            else:
                print(" GPS pos reliable:            False")

            if self.ref_pos_set:
                print(" Reference pos set:           True")
            else:
                print(" Reference pos set:           False")

            if self.time_synced:
                print(" GPS time synced:             True")
            else:
                print(" GPS time synced:             False")

            print(" System time:                 " + datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'))

            print(" Synced GPS time:             " + datetime.datetime.fromtimestamp(self.unix_time_with_offset).strftime('%Y-%m-%d %H:%M:%S'))

            print(" Reference pos geodetic:      Lat = {:.{}f}".format(self.reference_position_geo[0],6), ", Long = {:.{}f}".format(self.reference_position_geo[1],6))

            print(" Current pos geodetic:        Lat = {:.{}f}".format(self.current_pos_geo[0],6), ", Long = {:.{}f}".format(self.current_pos_geo[1],6))

            print(" Current pos cartesian:       X = {:.{}f}".format(self.current_pos_nav[0],2), ", Y = {:.{}f}".format(self.current_pos_nav[1],2))

            print(" Velocity magnitude:          V = {:.{}f}".format(self.current_velocity_magnitude,2))
            
            print(" ")

            print("########################################################################")

    ########################################################################################################

    ##################################### OTHER FUNCTIONS ##################################################
    # Updates system time
    def update_time(self, unix_time):
        command_string = 'sudo date -s @' + str(unix_time)
        os.system(command_string)

        #clk_id = time.CLOCK_REALTIME
        #time.clock_settime(clk_id, float(unix_time_as_string))

    # Calculates and publishes the marv pose, using the data from the SBG INS
    # Converts geodetic coordinates to local cartesian coordinates using the provided (needed before publising) reference position
    def publish_pose(self):
        if self.orientation_updated and self.position_updated:
            self.orientation_updated = False
            self.position_updated = False

            pose_message = PoseWithCovariance()
            # Set position (point)
            pose_message.pose.position.x = self.current_pos_nav[0]
            pose_message.pose.position.y = self.current_pos_nav[1]
            pose_message.pose.position.z = self.current_pos_nav[2]
            # Set orientation (quaternion)
            pose_message.pose.orientation.x = self.current_orientation[0] 
            pose_message.pose.orientation.y = self.current_orientation[1]
            pose_message.pose.orientation.z = self.current_orientation[2]
            pose_message.pose.orientation.w = self.current_orientation[3]
            # For and set covariance matrix
            pose_message.covariance.data = np.diag(np.hstack((self.current_pos_geo_accuracy,self.current_orientation_accuracy))).flatten()

            # Publish the pose
            self.marv_sbg_pose_publisher_.publish(pose_message)

            # Publish the velocity
            euler = self.quaternion_to_euler(self.current_orientation[0],self.current_orientation[1],self.current_orientation[2],self.current_orientation[3])
            theta = euler[2]
            R_ne2body = np.array([[np.cos(theta), np.sin(theta)],[-np.sin(theta), np.cos(theta)]])
            vel_ne = np.array([self.current_velocity_nav[0], self.current_velocity_nav[1]])
            vel_body = np.matmul(R_ne2body,vel_ne)
            current_velocity_message = Vector3()
            current_velocity_message.x = float(vel_body[0])
            current_velocity_message.y = float(vel_body[1])
            current_velocity_message.z = 0.0
            self.marv_sbg_velocity_publisher_.publish(current_velocity_message)

            # Publish raw geodetic position in lat,long,alt
            current_pos_message = Vector3() # Message for keeping the latitude, longitude and altitude message in Vector3()
            current_pos_message.x = self.current_pos_geo[0]
            current_pos_message.y = self.current_pos_geo[1]
            current_pos_message.z = self.current_pos_geo[2]
            self.marv_sbg_current_pos_publisher_.publish(current_pos_message)

    # Transforms geodetic coordinates in the WGS84 (World Geodetic System 1984) coordinate frame 
    # to local navigation coordinates in the ENU (East North Up) cordinate frame
    # Algorithm taken from https://www.researchgate.net/publication/27253833_Converting_GPS_coordinates_phi_lambda_h_to_navigation_coordinates_ENU
    # llh0: reference point in geodetic coordinates np.array(((lat),(long),(alt)))
    # llh:  data point in geodetic coordinates -||-
    def WGS84_to_ENU(self, llh0, llh):
        #self.get_logger().info("POS: " + str(llh0) + " " + str(llh))
        # Constants
        a = 6378137 # Length of earth's semi-major axis
        b = 6356752.3142 # Length of earth's semi-minor axis
        e2 = 1 - np.power(b/a,2)

        # Location of reference point in radians
        phi = np.deg2rad(llh0[0])
        lam = np.deg2rad(llh0[1])
        h = llh0[2]

        # Location of data points in radians
        dphi = np.deg2rad(llh[0]) - phi
        dlam = np.deg2rad(llh[1]) - lam
        dh = llh[2] - h

        # Definitions
        tmp1 = np.sqrt(1-e2*np.power(np.sin(phi),2))
        cl = np.cos(lam)
        sl = np.sin(lam)
        cp = np.cos(phi)
        sp = np.sin(phi)

        # Transformations
        de = (a/tmp1+h)*cp*dlam - (a*(1-e2)/(np.power(tmp1,3)) + h)*sp*dphi*dlam + cp*dlam*dh

        dn = (a*(1-e2)/np.power(tmp1,3) + h)*dphi + 1.5*cp*sp*a*e2*np.power(dphi,2) + np.power(sp,2)*dh*dphi + 0.5*sp*cp*(a/tmp1 + h)*np.power(dlam,2)

        du = dh - 0.5*(a-1.5*a*e2*np.power(cp,2) + 0.5*a*e2 + h)*np.power(dphi,2) - 0.5*np.power(cp,2)*(a/tmp1 - h)*np.power(dlam,2)

        return np.array(((de),(dn),(du)))

    # Transforms a ENU (East North Up) to a NED (North East Down) coordinate frame
    def ENU_to_NED(self, ENU_coords):
        NED_coords = np.array(((ENU_coords[1]),(ENU_coords[0]),(-ENU_coords[2])))
        
        return NED_coords

    # Transform Euler to Quaternions
    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]

    #Transform quaternions to Euler
    def quaternion_to_euler(self, x, y, z, w):
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x**2 + y**2)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        if abs(sinp) >=1:
            pitch = np.sign(sinp) * np.pi/2
        else:
            pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y**2 + z**2)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    ########################################################################################################

def main(args=None):
    rclpy.init(args=args)

    marv_sbg_interface = MARV_SBG_Interface()

    rclpy.spin(marv_sbg_interface)

    marv_sbg_interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
