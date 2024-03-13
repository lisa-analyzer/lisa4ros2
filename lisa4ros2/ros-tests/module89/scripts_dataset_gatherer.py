#!/usr/bin/env -S HOME=${HOME} ${HOME}/.virtualenvs/cv/bin/python

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, Point, Quaternion

import cv2, imutils
import tensorflow as tf
import math
import simplejson
import os, glob
import numpy as np

based_obj_points = np.array([[-0.2, -0.23, 0], [0.2, -0.23, 0], [0.2, 0.23, 0], [-0.2, 0.23, 0]])

camera_config = simplejson.load(open(os.path.join(get_package_share_directory('module89'), 'config', 'camera_config.json')))
cameraMatrix = np.array(camera_config['camera_matrix'], np.float32)
dist = np.array(camera_config['dist'])

dataset_config = simplejson.load(open(os.path.join(get_package_share_directory('module89'), 'config', 'dataset_config.json')))
output_path = dataset_config['capture_path']
file_list = glob.glob(os.path.join(output_path, '*'))
last_index = len(file_list)

four_points = []
canvas = None   # visualize assigned point
canvas_tmp = None # visualize moving cursor

def _bytes_feature(value):
    """Returns a bytes_list from a string / byte."""
    if isinstance(value, type(tf.constant(0))): # if value is tensor
        value = value.numpy() # get value of tensor
    return tf.train.Feature(bytes_list=tf.train.BytesList(value=[value]))

def _float_feature(value):
  """Returns a float_list from a float / double."""
  return tf.train.Feature(float_list=tf.train.FloatList(value=[value]))

def _int64_feature(value):
  """Returns an int64_list from a bool / enum / int / uint."""
  return tf.train.Feature(int64_list=tf.train.Int64List(value=[value]))

def image_example(image, rvec, tvec):   # Create a dictionary with features that may be relevant.
    feature = {
        'height': _int64_feature(image.shape[0]),
        'width': _int64_feature(image.shape[1]),
        'depth': _int64_feature(image.shape[2]),
        'image': _bytes_feature(tf.io.encode_png(image)),
        'rvec': tf.train.Feature(float_list=tf.train.FloatList(value=rvec.flatten())),
        'tvec': tf.train.Feature(float_list=tf.train.FloatList(value=tvec.flatten())),
        'camera_matrix': tf.train.Feature(float_list=tf.train.FloatList(value=cameraMatrix.flatten())),
        'dist': tf.train.Feature(float_list=tf.train.FloatList(value=dist.flatten())),
    }
    return tf.train.Example(features=tf.train.Features(feature=feature))

def rotate(rvec, angle):
    rotM = np.zeros(shape=(3, 3))
    new_rvec = np.zeros(3)
    rotM, _ = cv2.Rodrigues(rvec, rotM, jacobian=0)  # Convert from rotation vector -> rotation matrix (rvec=rot_vector, rotM=rot_matrix)
    rotM = np.dot(rotM, np.array([[math.cos(angle), -math.sin(angle), 0], [math.sin(angle), math.cos(angle), 0], [0, 0, 1]]))  # Rotate zeta degree about Z-axis
    new_rvec, _ = cv2.Rodrigues(rotM, new_rvec, jacobian=0)  # Convert from rotation matrix -> rotation vector
    return new_rvec

def click_corner(event, x, y, flags, param):
    global img, four_points, canvas, canvas_tmp
    if event == cv2.EVENT_LBUTTONDOWN:
        cv2.circle(canvas, (x, y), 5, (255, 0, 0), -1)
        four_points.append((x, y))
    if event == cv2.EVENT_MOUSEMOVE:
        canvas_tmp = canvas.copy()
        cv2.line(canvas_tmp, (x, 0), (x, 1080), (0, 255, 0), 1)
        cv2.line(canvas_tmp, (0, y), (1920, y), (0, 255, 0), 1)

class DatasetGatherer(Node):
    def __init__(self):
        super().__init__('dataset_gatherer')
        self.camera0_sub = self.create_subscription(Image, '/camera0/image', self.camera0_listener_callback, 10)
        self.camera1_sub = self.create_subscription(Image, '/camera1/image', self.camera1_listener_callback, 10)
        self.encoder_sub = self.create_subscription(Float32, '/chessboard/encoder', self.chessboard_encoder_callback, 10)

        self.bridge = CvBridge()  # Bridge between "CV (NumPy array)" <-> "ROS sensor_msgs/Image"
        self.top_chessboard_init_encoder, self.top_chessboard_init_pose = None, None  # Pair of encoder & pose used for reference (have same timestamp)
        self.side_chessboard_init_encoder, self.side_chessboard_init_pose = None, None  # Pair of encoder & pose used for reference (have same timestamp)
        self.chessboard_encoder, self.chessboard_pose = None, None  # encoder & pose in real-time (independent)

        ## Create timer to handle dataset saver
        self.timer = self.create_timer(0.05, self.timer_callback)   # 20 Hz
        self.top_frame = None
        self.side_frame = None

        ## State 0=not init, 1=init active, 2=already init
        self.top_frame_state = 0
        self.side_frame_state = 0
        self.init_canvas = None

        ## State 0=not capture, 1=capturing first half, 2=capturing back half
        self.encoder_culmulative = 0
        self.capturing = False
        self.last_chessboard_encoder = None
        self.writer = None


    def camera0_listener_callback(self, image):
        self.top_frame = self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
        # self.image_buffer['camera0'].append(self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough'))
        # if len(self.image_buffer['camera0']) > frame_buffer_length: self.image_buffer['camera0'] = self.image_buffer['camera0'][-frame_buffer_length:]

    def camera1_listener_callback(self, image):
        self.side_frame = self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
        # self.image_buffer['camera1'].append(self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough'))
        # if len(self.image_buffer['camera1']) > frame_buffer_length: self.image_buffer['camera1'] = self.image_buffer['camera1'][-frame_buffer_length:]

    def chessboard_encoder_callback(self, encoder):
        self.chessboard_encoder = encoder

    def timer_callback(self):
        global four_points, canvas, canvas_tmp, last_index
        if self.last_chessboard_encoder is not None:
            delta_rotation = self.chessboard_encoder.data - self.last_chessboard_encoder
            if delta_rotation < 0: delta_rotation += 2*math.pi
            self.encoder_culmulative += delta_rotation
        if self.chessboard_encoder is not None and self.top_frame is not None and self.side_frame is not None:
            visual_top, visual_side = self.top_frame.copy(), self.side_frame.copy()
            if self.top_chessboard_init_pose is not None:
                (rvec, tvec) = self.top_chessboard_init_pose
                delta_angle = self.chessboard_encoder.data - self.top_chessboard_init_encoder.data
                rvec = rotate(rvec, delta_angle)
                cv2.aruco.drawAxis(image=visual_top,
                                       cameraMatrix=cameraMatrix,
                                       distCoeffs=dist,
                                       rvec=rvec,
                                       tvec=tvec,
                                       length=0.1)
                if self.capturing:
                    tf_example = image_example(self.top_frame, rvec, tvec)
                    self.writer.write(tf_example.SerializeToString())

            if self.side_chessboard_init_pose is not None:
                (rvec, tvec) = self.side_chessboard_init_pose
                delta_angle = self.chessboard_encoder.data - self.side_chessboard_init_encoder.data
                rvec = rotate(rvec, delta_angle)
                cv2.aruco.drawAxis(image=visual_side,
                                   cameraMatrix=cameraMatrix,
                                   distCoeffs=dist,
                                   rvec=rvec,
                                   tvec=tvec,
                                   length=0.1)
                if self.capturing:
                    tf_example = image_example(self.side_frame, rvec, tvec)
                    self.writer.write(tf_example.SerializeToString())
            if self.encoder_culmulative > 2 * math.pi and self.capturing:
                self.capturing = False
                self.writer.close()
                last_index += 1

            cv2.imshow("Monitor", imutils.resize(cv2.hconcat([visual_top, visual_side]), width=1000))
            key = cv2.waitKey(1)
            if key == ord('1'):
                print('Init TOP')
                self.top_frame_state = 1
                self.top_chessboard_init_encoder = self.chessboard_encoder
                self.init_canvas = self.top_frame
                canvas = self.init_canvas.copy()
                four_points = []    # reset reference points buffer
            elif key == ord('2'):
                print('Init SIDE')
                self.side_frame_state = 1
                self.side_chessboard_init_encoder = self.chessboard_encoder
                self.init_canvas = self.side_frame
                canvas = self.init_canvas.copy()
                four_points = []    # reset reference points buffer
            if key == ord('1') or key == ord('2'):
                cv2.namedWindow('Assign Corner', cv2.WND_PROP_FULLSCREEN)
                cv2.setWindowProperty('Assign Corner', cv2.WND_PROP_AUTOSIZE, cv2.WINDOW_FULLSCREEN)
                cv2.setMouseCallback('Assign Corner', click_corner)
            if key == ord(' ') and self.top_chessboard_init_pose is not None and self.side_chessboard_init_pose is not None: # Start capture
                tfrecord_filename = os.path.join(output_path, f"{str(last_index).zfill(5)}.tfrecords")
                self.get_logger().info(tfrecord_filename)
                # Initiating the writer and creating the tfrecords file.
                self.writer = tf.io.TFRecordWriter(tfrecord_filename)
                self.encoder_culmulative = 0 # reset round counter
                self.capturing = True


        if self.top_frame_state == 1 or self.side_frame_state == 1:
            canvas_tmp = canvas.copy()
            cv2.imshow("Assign Corner", canvas_tmp)
            cv2.waitKey(1)
            if len(four_points) == 4:
                ret, rvec, tvec = cv2.solvePnP(objectPoints=based_obj_points,
                                               imagePoints=np.array(four_points, dtype=np.double),
                                               cameraMatrix=cameraMatrix,
                                               distCoeffs=dist,
                                               flags=0)
                if self.top_frame_state == 1:
                    self.top_frame_state = 2
                    self.top_chessboard_init_pose = (rvec, tvec)
                if self.side_frame_state == 1:
                    self.side_frame_state = 2
                    self.side_chessboard_init_pose = (rvec, tvec)
                cv2.destroyWindow('Assign Corner')
        if self.chessboard_encoder is not None: self.last_chessboard_encoder = self.chessboard_encoder.data

def main():
    rclpy.init()
    dataset_gatherer = DatasetGatherer()
    rclpy.spin(dataset_gatherer)
    rclpy.shutdown()

if __name__ == "__main__":
    main()