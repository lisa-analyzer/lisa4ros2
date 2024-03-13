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

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import asyncio

class LidarPreprocessingNode(Node):
    def __init__(self):

        super().__init__('plotter_node')

        #experimental
        self.subscriber_lidar = self.create_subscription(LaserScan, '/scan', self.cmd_scan_callback, 1) #/scan_filtered'

        self.lidar_message = LaserScan()

        # self.publisher_ = self.create_publisher(LaserScan, 'DEBUG', 10)
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.live_plot)

        # self.plot_loop = asyncio.get_event_loop()
        # self.plot_loop.run_until_complete(self.live_plot())



        self.fig, self.ax = plt.subplots()
        self.ln, = plt.plot([], [], 'ro')
        self.x_data, self.y_data = [] , []

        self.get_logger().info(f"SUCCESSFULLY CREATED THE PLOTTER NODE =====================")


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
    
    def plot_init(self):
            self.ax.set_xlim(0, 10000)
            self.ax.set_ylim(-7, 7)
            return self.ln


    async def live_plot(self):

        def animate(i):  # ser is the serial object

            lidar_scan = np.array(self.lidar_message.ranges.tolist())
            lidar_scan[~np.isfinite(lidar_scan)] = 8
            y = lidar_scan.tolist()

            self.get_logger().info(f"Hello THIS IS ITERATION {i} | mean : {lidar_scan[1:3]}")


            x = np.linspace(0,360,num=len(lidar_scan))
            
            # clear the last frame and draw the next frame
            ax.clear()
            ax.plot(x,y,linewidth=2, markersize=12)
            # Format plot
            # ax.set_ylim([0, 1050])
            ax.set_title("Potentiometer Reading Live Plot")
            ax.set_ylabel("Potentiometer Reading")



        fig, ax = plt.subplots()
        # time.sleep(2)

        # run the animation and show the figure
        ani = animation.FuncAnimation(
            fig, animate, frames=100000, interval=500 #, fargs=(x, y)
        )
        plt.show()

    def plot_scan(self):
        if self.lidar_message is not None:
            plt.clf()

            lidar_scan = np.array(self.lidar_message.ranges.tolist())
            lidar_scan[~np.isfinite(lidar_scan)] = 17
            y = lidar_scan.tolist()

            x = np.linspace(0,360,num=len(lidar_scan))
            
            # clear the last frame and draw the next frame
            plt.plot(x,y,linewidth=2, markersize=12)

            plt.draw()
            plt.pause(0.001)



    def cmd_scan_callback(self, message):
        self.lidar_message = message
        self.get_logger().info(f"========Updating LIDAR VALUE FOR PLOT ================")
        
        

        



def main(args=None):
    rclpy.init(args=args)

    lidar_preprocessing = LidarPreprocessingNode()


    while rclpy.ok():
        lidar_preprocessing.plot_scan()
        rclpy.spin_once(lidar_preprocessing)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)


    lidar_preprocessing.plot_loop.close()
    lidar_preprocessing.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
