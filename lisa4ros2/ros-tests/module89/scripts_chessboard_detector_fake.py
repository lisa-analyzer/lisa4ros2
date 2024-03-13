#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import UInt16MultiArray
from ament_index_python.packages import get_package_share_directory
from module89.srv import ChessboardDetection


import os
import simplejson
import numpy as np
import cv2

class ChessboardDetectorService(Node):
    def __init__(self):
        super().__init__('chessboard_detector_fake')
        self.srv = self.create_service(ChessboardDetection, 'chessboard_detection', self.detect_callback)
        fake_data = simplejson.load(open(os.path.join(get_package_share_directory('module89'), 'models', '00001.json')))
        x2, y2 = fake_data['objects'][0]['bounding_box']['bottom_right']
        x1, y1 = fake_data['objects'][0]['bounding_box']['top_left']
        height = int(fake_data['camera_data']['height'])
        width = int(fake_data['camera_data']['width'])
        if x1 < 0: x1 = 0
        if x2 >= width: x2 = width-1
        if y1 < 0: y1 = 0
        if y2 >= height: y2 = height-1
        self.fake_data = UInt16MultiArray()
        self.fake_data.data = [int(x1), int(y1), int(x2), int(y2)]
        self.bridge = CvBridge()
    def detect_callback(self, request, response):
        img = self.bridge.imgmsg_to_cv2(request.img, "bgr8")
        # cv2.imshow("detection", img)
        # cv2.waitKey(0)
        # cv2.destroyWindow("detection")
        self.get_logger().info("BBBBBBBBBBBBBBBBBB")
        response.bbox = self.fake_data
        return response

def main():
    rclpy.init()
    detection_service = ChessboardDetectorService()
    rclpy.spin(detection_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()