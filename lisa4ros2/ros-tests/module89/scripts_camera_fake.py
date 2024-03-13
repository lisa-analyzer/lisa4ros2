#!/usr/bin/env -S HOME=${HOME} ${HOME}/.virtualenvs/cv/bin/python

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import CameraInfo

import cv2
import v4l2capture
import select
import os
import numpy as np
import subprocess
import simplejson

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('fake_camera_publisher')
        self.publisher_ = self.create_publisher(Image, '/image', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/info', 10)
        timer_period = 1/30  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.config = simplejson.load(open(os.path.join(get_package_share_directory('module89'), 'config', 'camera_config.json')))
        print(self.config)
        self.frame = cv2.imread(os.path.join(get_package_share_directory('module89'), 'models', '00001.png'))
        self.bridge = CvBridge()

    def timer_callback(self):
        # Publish camera info
        camera_info = CameraInfo()
        camera_info.height = self.config['height']
        camera_info.width = self.config['width']
        camera_info.d = self.config['dist']
        camera_info.k = sum(self.config['camera_matrix'], [])   # Flatten [[3], [3], [3]] to [9]
        self.camera_info_pub.publish(camera_info)
        # Publish image
        image_msg = self.bridge.cv2_to_imgmsg(self.frame, "bgr8")
        self.publisher_.publish(image_msg)
        # self.get_logger().info('Publishing image')

def main():
    rclpy.init()
    camera_service = CameraPublisher()
    rclpy.spin(camera_service)

    camera_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()