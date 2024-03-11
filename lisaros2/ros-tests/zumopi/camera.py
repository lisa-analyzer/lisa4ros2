#!/usr/bin/env python3

#  Telemetry Project:
#  Renato and Michael.

# -------------IMPORTS-------------------
import rclpy
from rclpy.node import Node
import cv2
import os
from time import time

from telemetry_interfaces.msg import (
  Bool, Int, String, CompressedImage, CameraSettings, SamplingRate, EnableRead
)

from telemetry_pkg.utilities import RepeatedTimer, TelemetryLogger, generate_str_timestamp, log_msg

# -------------GLOBAL PARAMETERS-----------
PUBLISH_PERIOD = 0.005 # sec
CAMERA_KEEP_ALIVE_PERIOD = 2 # sec
SENSOR_SETTINGS_PERIOD = 2 # sec
JPEG_QUALITY = 60
PERIOD_INIT_CAMERA = 100 # msec
ENABLE_INIT_CAMERA = True
FRAME_WIDTH = 320
FRAME_HEIGHT = 240


class CameraNode(Node):
  def __init__(self):
    super().__init__("camera")
    # camera parameters
    self.camera_period = PERIOD_INIT_CAMERA
    self.enable_camera = ENABLE_INIT_CAMERA
    self.camera_last_sample_time = 0
    self.is_camera_open = False

    self.publish_last_sample_time = 0             # last time entered publish function
    self.previous_publish_last_sample_time = 0    # used to check if there is a diff in the times
  
    self.capture = None

    self.init_log()

    # Publishers
    self.camera_publisher_ = self.create_publisher(CompressedImage, "camera_frame", 10)
    self.data_camera_publisher_ = self.create_publisher(CameraSettings, "camera_sensor_settings", 10)
    self.keep_alive_publisher_ = self.create_publisher(String, "keep_alive", 10)
    
    # Sampling Period msec
    self.period_camera_subscriber_ = self.create_subscription(SamplingRate, "set_period", self.callback_set_camera_period, 10)

    # Enable sensor reading
    self.enable_camera_subscriber_ = self.create_subscription(EnableRead, "enable_read", self.callback_enable_camera_read, 10)

    self.logger.info("Started camera node")

  # -------------LOGGER------------
  def init_log(self):
    log_dir = os.path.join(os.path.expanduser('~'), "ros2_ws/telemetry_logs")
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
    log_file_path = os.path.join(log_dir, f"camera_log_{generate_str_timestamp()}.txt")

    self.logger = TelemetryLogger('logger', log_file_path).logger
    self.log_level_subscriber_ = self.create_subscription(Int, "log_level", self.callback_set_log_level, 10)

    self.telemetry_data_logger = TelemetryLogger('telemetry_data_logger', log_file_path, console=False, disabled=True).logger                
    self.telemetry_data_log_enable_subscriber_ = self.create_subscription(Bool, "telemetry_data_log_enable", self.callback_telemetry_data_log_enable,10)

  def callback_set_log_level(self, msg):
    self.logger.setLevel(msg.data)

  def callback_telemetry_data_log_enable(self, msg):
    if msg.data:
        self.telemetry_data_logger.disabled = False
    else:
        self.telemetry_data_logger.disabled = True

  def publish(self):
    self.publish_last_sample_time = time()

    if not self.is_camera_open:
      return

    if not self.enable_camera:
      return

    if (time() * 1000 - self.camera_last_sample_time) >= self.camera_period:
      self.camera_last_sample_time = time() * 1000

      # reads in BGR
      ret, frame = self.capture.read()

      if ret == False:
        self.logger.warn("Could not read from camera")
        return

      encoded_frame = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])[1]
      encoded_list = encoded_frame.tolist()
      
      msg = CompressedImage()
      msg.compressed_image = encoded_list

      self.camera_publisher_.publish(msg)

  def callback_set_camera_period(self, msg):
    if msg.period_msec <= 0:
        self.logger.warn("Sampling period must be positive")
        return

    if msg.sensor == "camera":
        self.camera_period = msg.period_msec
    else:
        self.logger.warn(f"Recieved a sensor which is not camera in set period: {msg.sensor}")
        return
    
    log_msg(self.telemetry_data_logger, "recieved", msg)

  def callback_enable_camera_read(self, msg):
    if msg.sensor == "camera":
        self.enable_camera = bool(msg.enable_read)
    else:
        self.logger.warn(f"Recieved a sensor which is not camera in set enable: {msg.sensor}")
        return
    
    log_msg(self.telemetry_data_logger, "recieved", msg)

  def sensor_settings_publish(self):
      msg = CameraSettings()
      msg.period_msec = self.camera_period
      msg.enable_read = self.enable_camera
      self.data_camera_publisher_.publish(msg)
      log_msg(self.telemetry_data_logger, "published", msg)

  def camera_keep_alive_publish(self):
    if not self.is_camera_open:
      self.logger.warn(f"camera is not open")
      return

    # if didn't eneter some time to publish, something is wrong, don't send keep alive of camera
    if self.previous_publish_last_sample_time - self.publish_last_sample_time < 0:
      try:
        self.capture.getBackendName() # try access camera
        msg = String()
        msg.data = "camera"
        self.keep_alive_publisher_.publish(msg)
      except:
        self.logger.warn(f"camera stopped working")
    else:
        self.logger.warn(f"stopped publishing frames for some reason")

    self.previous_publish_last_sample_time = self.publish_last_sample_time

def main(args=None):
  rclpy.init(args=args) # Initialize ROS2 libraries
  node = CameraNode()
  node.capture = cv2.VideoCapture(0)
  node.capture.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
  node.capture.set(cv2.CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT)
  node.is_camera_open = node.capture.isOpened()

  try:
      publish_timer = RepeatedTimer(PUBLISH_PERIOD, node.publish)
      sensor_settings_timer = RepeatedTimer(SENSOR_SETTINGS_PERIOD, node.sensor_settings_publish)
      camera_keep_alive_timer = RepeatedTimer(CAMERA_KEEP_ALIVE_PERIOD, node.camera_keep_alive_publish)
      rclpy.spin(node) # Make node run in background

  except KeyboardInterrupt:
      node.logger.info("Camera got Keyboard Interrupt")
      pass

  finally: 
      if node.is_camera_open:
        node.capture.release()
      publish_timer.stop()
      sensor_settings_timer.stop()
      camera_keep_alive_timer.stop()
      node.logger.info("Shutting down camera")
      node.destroy_node()

if __name__ == '__main__':
  main()
