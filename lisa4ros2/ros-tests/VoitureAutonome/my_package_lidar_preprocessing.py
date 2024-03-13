from asyncio import sleep
import rclpy
from interfaces.msg import MonAckermannDrive
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from std_msgs.msg import Bool
from interfaces.action import DriverAction
from interfaces.msg import DrivingInstruction
from sensor_msgs.msg import LaserScan
from asyncio import Future
from collections import deque
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import numpy as np
import time

class LidarPreprocessingNode(Node):
    def __init__(self):

        super().__init__('lidar_preprocessing_node')

        self.timeOutCounter = 0
        timer_period = 1  # seconds
        self.publisher_connexion = self.create_publisher(LaserScan, 'LidarScanPreprocessed', 1)


        #experimental
        self.subscriber_lidar = self.create_subscription(LaserScan, '/scan', self.cmd_scan_callback, 1) #/scan_filtered'

        #### Parameters

        self.sampling_step = 1

        self.aperture_half_angle = 75

        self.min_range = 0.15

        self.filter_resolution = 180

        self.aperture_half_angle_rad = self.aperture_half_angle * np.pi / 180
        


        # TODO: aperture half angle is not supposed to be here, it is supposes to be set in the driver

    def nan_helper(self,y):
            """Helper to handle indices and logical indices of NaNs.
            Input:
                - y, 1d numpy array with possible NaNs
            Output:
                - nans, logical indices of NaNs
                - index, a function, with signature indices= index(logical_indices),
                to convert logical indices of NaNs to 'equivalent' indices
            Example:
                >>> # linear interpolation of NaNs
                >>> nans, x= nan_helper(y)
                >>> y[nans]= np.interp(x(nans), x(~nans), y[~nans])
            """
            return np.isnan(y), lambda z: z.nonzero()[0]
    
    def cmd_scan_callback(self, message):
        
        t0 = time.time()
        lidar_scan = message.ranges #TODO: est ce que c'est correct ca ?

        raw_lidar = np.array(lidar_scan.tolist())

        raw_lidar[~np.isfinite(raw_lidar)] = np.NaN
        # raw_lidar[~np.isfinite(raw_lidar)] = 0.15
        

        # filtered_lidar = np.array_split(raw_lidar,self.filter_resolution)
        
        nans, x= self.nan_helper(raw_lidar)
        raw_lidar[nans]= np.interp(x(nans), x(~nans), raw_lidar[~nans])
        
        filtered_lidar = raw_lidar.tolist()
        
#         filtered_lidar = [np.median(chunk) for chunk in filtered_lidar]





        angular_resolution_deg = 360 / len(filtered_lidar)
        angular_resolution_rad = 2 * np.pi / len(filtered_lidar)

        aperture = int(self.aperture_half_angle / angular_resolution_deg)

        lidar = filtered_lidar[-aperture:] + filtered_lidar[:aperture + 1]

        if self.sampling_step > 1:
            sampled_lidar = lidar[::self.sampling_step]
        else:
            sampled_lidar = lidar

        # self.get_logger().info("Sampled Lidar: "+str(sampled_lidar))

        # self.get_logger().info(f"min : {message.angle_min} | max: {message.angle_max} | inc {message.angle_increment}")

        t1 = time.time()
        # self.get_logger().info(f"time preprocessing:{t1-t0}")


        #msg = LaserScan()
        msg = message
        msg.header.frame_id = 'frame_preprocessed'
        msg.ranges = sampled_lidar
        msg.angle_min = self.aperture_half_angle_rad
        msg.angle_max = -self.aperture_half_angle_rad
        msg.angle_increment = message.angle_increment

        self.publisher_connexion.publish(msg)









    # def step(self):
    #     rclpy.spin_once(self.__node, timeout_sec=0)
    #     if self._current_goal_future is not None:
    #         rclpy.spin_until_future_complete(self.__node, self._current_goal_future)
    #         self._current_goal_future = None


def main(args=None):
    rclpy.init(args=args)

    lidar_preprocessing = LidarPreprocessingNode()

    rclpy.spin(lidar_preprocessing)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lidar_preprocessing.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
