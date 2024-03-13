import sys
import json
import numpy as np
import cv2

from array import array

from solar_interfaces.srv import DeliverImg 
import rclpy
from rclpy.node import Node

_RGB = 1
class DeliverImgClient(Node):

    def __init__(self):
        super().__init__('deliver_client_async')
        self.cli = self.create_client(DeliverImg , 'deliver_server')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = DeliverImg .Request()

    def send_request(self, id):
        self.req.photo_id = id
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    try:
        rclpy.init()

        minimal_client = DeliverImgClient()
        response = minimal_client.send_request(int(sys.argv[1]))
        
        minimal_client.get_logger().info(
        f'respuesta { response.photo}')
        if response.photo != array('B'):
            img_bytes = np.array(response.photo, dtype=np.uint8)
            img = cv2.imdecode(img_bytes, _RGB)
            cv2.imshow("deliver_client", img)
            cv2.waitKey(0)
        else:
            minimal_client.get_logger().info(
        f'arreglo vacio')
    finally:
        minimal_client.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()