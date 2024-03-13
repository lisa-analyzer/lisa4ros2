#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import Float32
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import CameraInfo, Image
from module89.msg import ChessboardImgPose

import cv2
import math
import simplejson
import os
import numpy as np

# config = simplejson.load(open(os.path.join(get_package_share_directory('module89'), 'config', 'camera_config.json')))
# cameraMatrix = np.array(config['camera_matrix'], np.float32)
# dist = np.array(config['dist'])
frame_buffer_length = 10

class ChessboardTracker(Node):
    def __init__(self):
        super().__init__('chessboard_tracker')
        self.image_buffer = {'camera0':[], 'camera1':[]}  # image buffer
        # Create camera_image, chessboard_encoder subscriber
        self.camera0_sub = self.create_subscription(Image, '/camera0/image', self.camera0_listener_callback, 10)
        self.camera1_sub = self.create_subscription(Image, '/camera1/image', self.camera1_listener_callback, 10)
        self.encoder_sub = self.create_subscription(Float32, '/chessboard/encoder', self.chessboard_encoder_callback, 10)
        self.dope0_output = self.create_subscription(Pose, '/dope0/output', self.dope0_output_callback, 10)
        self.dope1_output = self.create_subscription(Pose, '/dope1/output', self.dope1_output_callback, 10)

        self.dope0_input = self.create_publisher(Image, '/dope0/input', 10)
        self.dope1_input = self.create_publisher(Image, '/dope1/input', 10)
        self.top_pose_pub = self.create_publisher(ChessboardImgPose, '/chessboard/top/ImgPose', 10)
        self.side_pose_pub = self.create_publisher(ChessboardImgPose, '/chessboard/side/ImgPose', 10)
        
        self.bridge = CvBridge()    # Bridge between "CV (NumPy array)" <-> "ROS sensor_msgs/Image"
        self.chessboard_init_encoder, self.chessboard_init_pose = None, None    # Pair of encoder & pose used for reference (have same timestamp)
        self.chessboard_encoder, self.chessboard_pose = None, None  # encoder & pose in real-time (independent)

        # Camera matrix (needed for solving PnP problem)
        self.camera0_info = {'width':None, 'height':None, 'cameraMatrix':None, 'dist':None}
        self.camera1_info = {'width':None, 'height':None, 'cameraMatrix':None, 'dist':None}

        ## Create timer to handle pipeline feeding
        # self.timer = self.create_timer(0.05, self.timer_callback)   # 20 Hz
        self.top_frame = np.array((480, 640, 3))
        self.side_frame = np.array((480, 640, 3))
        self._last_time = time.time()
    def camera0_listener_callback(self, image):
        self.dope0_input.publish(image)
        self.top_frame = self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
        # self.image_buffer['camera0'].append(self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough'))
        # if len(self.image_buffer['camera0']) > frame_buffer_length: self.image_buffer['camera0'] = self.image_buffer['camera0'][-frame_buffer_length:]

    def camera1_listener_callback(self, image):
        self.dope1_input.publish(image)
        self.side_frame = self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
        # self.image_buffer['camera1'].append(self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough'))
        # if len(self.image_buffer['camera1']) > frame_buffer_length: self.image_buffer['camera1'] = self.image_buffer['camera1'][-frame_buffer_length:]
    def dope0_output_callback(self, pose):
        # Publish result to /chessboard/XXX/ImgPose
        # self.get_logger().info("DOPE output: " + str(pose))
        img_pose_msg = ChessboardImgPose()
        img_pose_msg.pose = pose
        img_pose_msg.image = self.bridge.cv2_to_imgmsg(self.top_frame, "bgr8")
        # pose_pub = self.top_pose_pub if self._last_cam == 0 else self.side_pose_pub
        pose_pub = self.top_pose_pub
        pose_pub.publish(img_pose_msg)

    def dope1_output_callback(self, pose):
        img_pose_msg = ChessboardImgPose()
        img_pose_msg.pose = pose
        img_pose_msg.image = self.bridge.cv2_to_imgmsg(self.side_frame, "bgr8")
        pose_pub = self.side_pose_pub
        pose_pub.publish(img_pose_msg)
    def chessboard_encoder_callback(self, encoder):
        pass
        
    # def timer_callback(self):
        # self.get_logger().info("_in_pipeline: " + str(self._in_pipeline))
        # self._last_time = time.time()
        # if self._last_cam == 1:             # Ready to send image from camera0
        #     self._last_cam = 0              # Set recent cam to this cam
        #     if len(self.image_buffer['camera0']) > 0:
        #         self._last_image = self.image_buffer['camera0'].pop(-1) # Get most recent image
        #         self._in_pipeline = True
        #         self.dope0_input.publish(self.bridge.cv2_to_imgmsg(self._last_image, "bgr8"))
        # elif self._last_cam == 0:           # Ready to send image from camera1
        #     self._last_cam = 1              # Set recent cam to this cam
        #     if len(self.image_buffer['camera1']) > 0:
        #         self._last_image = self.image_buffer['camera1'].pop(-1) # Get most recent image
        #         self._in_pipeline = True
        #         self.dope1_input.publish(self.bridge.cv2_to_imgmsg(self._last_image, "bgr8"))

            

    

def main():
    rclpy.init()
    chessboard_detector = ChessboardTracker()
    rclpy.spin(chessboard_detector)
    # chessboard_detector.destroy_subscription(chessboard_detector.camera_sub) # Not need camera after init pose
    rclpy.shutdown()

if __name__ == "__main__":
    main()