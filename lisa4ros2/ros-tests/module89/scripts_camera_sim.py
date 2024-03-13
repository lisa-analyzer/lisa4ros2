#!/usr/bin/env -S HOME=${HOME} ${HOME}/.virtualenvs/cv/bin/python
import random

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, Point, Quaternion
from module89.msg import ChessboardImgPose
from ament_index_python.packages import get_package_share_directory

import os
import glob
import json
import numpy as np
import tensorflow as tf
import cv2
import math
from scipy.spatial.transform import Rotation as R

## Avoid to use all GPU(s)VRAM
gpus = tf.config.experimental.list_physical_devices('GPU')
for gpu in gpus: tf.config.experimental.set_memory_growth(gpu, True)

# Converting the values into features
def _int64_feature(value):  # _int64 is used for numeric values
    return tf.train.Feature(int64_list=tf.train.Int64List(value=value))
def _bytes_feature(value):  # _bytes is used for string/char values
    return tf.train.Feature(bytes_list=tf.train.BytesList(value=[value]))
def _float_feature(value):
    return tf.train.Feature(float_list=tf.train.FloatList(value=value))

# Create a dictionary describing the features.
image_feature_description = {
    'height': tf.io.FixedLenFeature([], tf.int64),
    'width': tf.io.FixedLenFeature([], tf.int64),
    'depth': tf.io.FixedLenFeature([], tf.int64),
    'image': tf.io.FixedLenFeature([], tf.string),
    "rvec": tf.io.FixedLenFeature([3], tf.float32),
    "tvec": tf.io.FixedLenFeature([3], tf.float32),
    "camera_matrix": tf.io.FixedLenFeature([9], tf.float32),
    'dist': tf.io.FixedLenFeature([5], tf.float32)
}

def _parse_image_function(example_proto):
  # Parse the input tf.train.Example proto using the dictionary above.
  return tf.io.parse_single_example(example_proto, image_feature_description)

def preparePose(rvec, tvec):
    tvec = tvec.reshape((-1, 1))
    pose_msg = Pose()
    pose_msg.position = Point(x=float(tvec[0][0]),
                              y=float(tvec[1][0]),
                              z=float(tvec[2][0]))
    r = R.from_matrix(cv2.Rodrigues(rvec)[0])
    rvec = Quaternion()
    [rvec.x, rvec.y, rvec.z, rvec.w] = r.as_quat()
    pose_msg.orientation = rvec
    return pose_msg
def pose2view_angle(rvec, tvec):
    rotM = np.zeros(shape=(3, 3))
    rotM, _ = cv2.Rodrigues(rvec, rotM, jacobian=0)
    tvec_tile_final = np.dot(tvec, rotM.T).reshape(3)
    tile_x, tile_y, tile_z = tvec_tile_final[0], tvec_tile_final[1], tvec_tile_final[2]
    angle_rad = math.asin((math.sqrt(tile_x ** 2 + tile_y ** 2)) / (math.sqrt(tile_x ** 2 + tile_y ** 2 + tile_z ** 2)))
    return angle_rad
class CameraPublisherSim(Node):
    def __init__(self):
        super().__init__('camera_publisher_sim')
        self.cam_config = json.load(open(os.path.join(get_package_share_directory('module89'), 'config', 'camera_config.json')))
        self.data_config = json.load(open(os.path.join(get_package_share_directory('module89'), 'config', 'dataset_config.json')))
        self.top_pose_pub = self.create_publisher(ChessboardImgPose, '/chessboard/top/ImgPose', 10)
        self.side_pose_pub = self.create_publisher(ChessboardImgPose, '/chessboard/side/ImgPose', 10)
        # self.info0_publisher = self.create_publisher(CameraInfo, '/camera0/info', 10)
        # self.info1_publisher = self.create_publisher(CameraInfo, '/camera1/info', 10)

        timer_period = 1 / 5  # seconds
        self.top_timer = self.create_timer(timer_period, self.top_timer_callback)
        self.side_timer = self.create_timer(timer_period, self.side_timer_callback)
        self.bridge = CvBridge()

        # Prepare camera info
        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.width = self.cam_config["width"]
        self.camera_info_msg.height = self.cam_config["height"]
        self.camera_info_msg.k = sum(self.cam_config["camera_matrix"], [])  # Serialize camera matrix to float32[9]
        self.camera_info_msg.d = self.cam_config["dist"]

        # Prepare *.tfrecord
        self.image_top_buffer = []
        self.image_side_buffer = []
        self.pose_top_buffer = []
        self.pose_side_buffer = []
        output_path = self.data_config['capture_path']
        raw_file_list = sorted(glob.glob(os.path.join(output_path, '*.tfrecords')))
        self.file_list = []
        for file in raw_file_list: # select only file with numeric name (raw vdo)
            if os.path.basename(file)[0].isnumeric(): self.file_list.append(file)
        print(self.file_list)
        for i in random.choices(range(len(self.file_list)-1), k=2):
        # for i in [len(self.file_list) - 1]:
            file_path = self.file_list[i]
            dataset = tf.data.TFRecordDataset(file_path)
            parsed_image_dataset = dataset.map(_parse_image_function)

            for image_features in parsed_image_dataset:
                # height = image_features['height'].numpy()
                # width = image_features['width'].numpy()
                # depth = image_features['depth'].numpy()
                image = tf.io.decode_png(image_features['image'])  # Auto detect image shape when decoded
                image = np.array(image, dtype=np.uint8)
                rvec = image_features['rvec'].numpy()
                tvec = image_features['tvec'].numpy()
                angle = pose2view_angle(rvec, tvec)  # get view angle (radian)
                if angle > 0.2: # side view
                    self.image_side_buffer.append(image)
                    self.pose_side_buffer.append((rvec, tvec))
                else:           # top view
                    self.image_top_buffer.append(image)
                    self.pose_top_buffer.append((rvec, tvec))
        self.side_frame_index, self.side_top_index = 0, 0
    def top_timer_callback(self):
        image = self.image_top_buffer[self.side_top_index]
        (rvec, tvec) = self.pose_top_buffer[self.side_top_index]
        self.side_top_index += 1
        if self.side_top_index == len(self.image_top_buffer): self.side_top_index = 0 # reset after exceed vdo length

        img_pose_msg = ChessboardImgPose()
        img_pose_msg.pose = preparePose(rvec, tvec)
        img_pose_msg.image = self.bridge.cv2_to_imgmsg(image, "bgr8")
        self.top_pose_pub.publish(img_pose_msg)
    def side_timer_callback(self):
        image = self.image_side_buffer[self.side_frame_index]
        (rvec, tvec) = self.pose_side_buffer[self.side_frame_index]
        self.side_frame_index += 1
        if self.side_frame_index == len(self.image_side_buffer): self.side_frame_index = 0  # reset after exceed vdo length

        img_pose_msg = ChessboardImgPose()
        img_pose_msg.pose = preparePose(rvec, tvec)
        img_pose_msg.image = self.bridge.cv2_to_imgmsg(image, "bgr8")
        self.side_pose_pub.publish(img_pose_msg)

def main():
    rclpy.init()
    camera_service = CameraPublisherSim()
    rclpy.spin(camera_service)
    camera_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()