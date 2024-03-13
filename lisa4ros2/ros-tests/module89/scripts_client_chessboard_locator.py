#!/usr/bin/env -S HOME=${HOME} ${HOME}/.virtualenvs/cv/bin/python
import cv2, os
from module89.srv import ChessboardPose

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge

class TestChessboardLocator(Node):
    def __init__(self):
        super().__init__('test_chessboard_locator')
        self.cli = self.create_client(ChessboardPose, 'chessboard_locator')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for service ...")
        self.req = ChessboardPose.Request()
        self.bridge = CvBridge()

    def send_request(self):
        frame = cv2.imread(os.path.join(get_package_share_directory('module89'), 'models', '00001.png'))
        self.req.img = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        self.future = self.cli.call_async(self.req)
def main():
    rclpy.init()

    test_client = TestChessboardLocator()
    test_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(test_client)
        if test_client.future.done():
            try:
                response = test_client.future.result()
            except Exception as e:
                test_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                test_client.get_logger().info('Result: %s'%(str(response.init_pose)))
            break
    test_client.get_logger().info("Finish ... Destroy Node")
    test_client.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()