import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool
from interfaces.msg import MonAckermannDrive
from sensor_msgs.msg import LaserScan
import numpy as np
from .aeb import BasicAEB

import time


class BrainNode(Node):

    def __init__(self):
        super().__init__('brain_node')
        self.isConnected = False
        self.isRaceOn = False
        self.subscriber_connexion = self.create_subscription(Bool, 'IsConnected', self.getMessageConnexion, 10)
        self.subscriber_race = self.create_subscription(Bool, 'IsRaceOn', self.getMessageRace, 10)

        self.publisher_connexion = self.create_publisher(MonAckermannDrive, 'BrainDrivingCommand', 1)
        
        self.subscriber_lidar = self.create_subscription(LaserScan, 'LidarBrain', self.execute_callback, 1)

        ## TO parameterized
        safety_bubble_r = 0.029 * 10#0.029
        aperture_half_angle = 75
        disparity_threshold = 0.8
        aperture_tight_front_half_angle_deg = 45
        aeb_ttc_threshold = 0.5
        aeb_lidar_threshold = .022

        ### CONFIG ###
        self.safety_bubble_r = safety_bubble_r

        self.aperture_half_angle = aperture_half_angle
        self.disparity_threshold = disparity_threshold
        self.aperture_tight_front_half_angle_deg = aperture_tight_front_half_angle_deg
        self.aeb_ttc_threshold = aeb_ttc_threshold
        self.aeb_lidar_threshold = aeb_lidar_threshold
        self.aeb_retro_speed = -3.16587354 #very weird speed to get an AEB signature. Pretty ugly but it works :)

        self.speed_profile = {'dist_threshold':[0.5,1.2,2.0,15.],'speed':[0.5,1.2,1.6,2.5]}

        self.aeb = BasicAEB()

        self.max_steering_angle = 0.278 
        self.min_range = 0.15

        self.get_logger().info(str("Successfully created a brain node with MTD driving policy"))

    def execute_callback(self, lidar_scan):
        lidar_scan = lidar_scan.ranges
        #############################################
        t0 = time.time()
        lidar = np.array(lidar_scan.tolist())


        #TODO A CHANGER CETTE MERDE HARDCODÉ HORRIBLE ALERTE ALERTE
        half_angle_deg = -75
        half_angle_rad = -75 * np.pi/180
        angular_resolution_deg = 75*2/len(lidar)
        angular_resolution_rad = angular_resolution_deg * np.pi/180



        # Initialized with a value because if the original value of a point is lower than all the rest than it must stay true
        virtuals = [np.array(lidar)]
        for i in range(len(lidar) - 1):
            if abs(lidar[i] - lidar[i + 1]) > self.disparity_threshold:  # check if it's a disparity

                # is the disparity left to right or right to left ?
                idx_obstacle = i if lidar[i] - lidar[i + 1] < 0 else i + 1

                virtual_lidar = np.array(lidar)

                # Compute the distance for safety bubble filtering
                dists = [2 * lidar[idx_obstacle] * abs(np.sin(abs(idx_obstacle - j) * angular_resolution_rad / 2))
                         for j in range(len(lidar))]
                mask = [j <= self.safety_bubble_r for j in dists]

                # All lidar points in a certain area are set to the value of the disparity point
                virtual_lidar[mask] = lidar[idx_obstacle]
                virtuals.append(virtual_lidar)

        if len(virtuals) == 1:
            virtual_lidar = np.array(lidar)
        else:
            virtual_lidar = np.stack(virtuals, axis=1)
            virtual_lidar = np.min(virtual_lidar, axis=1)

        virtual_lidar = virtual_lidar.tolist()


        # Compute the angle of the goal both in radiant and degrees.
        idx_goal = virtual_lidar.index(max(virtual_lidar))





        angle_goal_deg =  half_angle_deg + idx_goal * angular_resolution_deg
        angle_goal_rad =  half_angle_rad + idx_goal * angular_resolution_rad

        self.get_logger().info("num discrepancy:" + str(len(virtuals)-1))

        # print("Angle GOAL : ", angle_goal_deg)
        # print("Angle GOAL RAD: ", angle_goal_rad)

        aperture_tight_front = int((-half_angle_deg - self.aperture_tight_front_half_angle_deg) / angular_resolution_deg)

        lidar_tight_front = lidar[aperture_tight_front:-aperture_tight_front]
        
        
        self.get_logger().info(f"size tight {len(lidar_tight_front)} | aperture {aperture_tight_front} | size lidar {len(lidar)}")

        # print("---TTC FRONT", ttc_tight_front)

        direction = np.sign(angle_goal_deg)
        steeringCommand = direction * min(self.max_steering_angle, np.abs(angle_goal_rad) * 1.8)
        speedCommand = self.dist_to_speed(np.quantile(lidar_tight_front, q=0.7))

        self.get_logger().info("Angle Goal:" + str(angle_goal_deg))
#         self.get_logger().info("LIDARRRRR:" + str(lidar[::2]))

#         self.get_logger().info("QUANTILE 1:" + str(np.quantile(lidar_tight_front, q=0.7)))
        # self.get_logger().info("QUANTILE 2:" + str(np.quantile(lidar_tight_front, q=0.4)))

        speedCommand, steeringCommand,is_braking = self.aeb.check(speedCommand, steeringCommand, lidar_tight_front,
                                                       direction)
        
        if is_braking:
            self.get_logger().info(f"EMERGENCY BREAKING")

        self.get_logger().info(f"SIZE LIDAR TIGHT FRONT: {len(lidar_tight_front)}")                    
#         self.get_logger().info((lidar_tight_front[0::15]).__str__())
        self.get_logger().info(f"isConnected {self.isConnected} | isRaceOn: {self.isRaceOn} | speed: {speedCommand} | angle:{steeringCommand}")
        
        t1 = time.time()
        self.get_logger().info(f"Executed in {t1-t0}s")

        msg = MonAckermannDrive()
        if(self.isRaceOn and self.isConnected):
            msg.speed = float(speedCommand)
            msg.steering_angle = float(steeringCommand)
        else:
            msg.speed = float(0)
            msg.steering_angle = float(0)

        self.publisher_connexion.publish(msg)

    def dist_to_speed(self, min_ttc):
        for i, ttc_thresh in enumerate(self.speed_profile["dist_threshold"]):
            if min_ttc <= ttc_thresh:
                return self.speed_profile["speed"][i]
        return self.speed_profile["speed"][-1]


    def getMessageRace(self, message: Bool):
        if(message.data):
            self.isRaceOn = True
        else:
            self.isRaceOn = False

    def getMessageConnexion(self, message: Bool):
        if(message.data):
            self.isConnected = True
        else:
            self.isConnected = False

def main(args=None):
    rclpy.init(args=args)

    brain_node = BrainNode()

    rclpy.spin(brain_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    brain_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
