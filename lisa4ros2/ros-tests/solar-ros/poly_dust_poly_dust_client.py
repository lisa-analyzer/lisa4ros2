import sys
import json
import numpy as np
import cv2
from array import array

from solar_interfaces.srv import DeliverImg 
import rclpy
from rclpy.node import Node

_RGB = 1
_image_size = (1024, 1792)
class DeliverImgClient(Node):

    def __init__(self):
        super().__init__('poly_dust_client')
        self.cli = self.create_client(DeliverImg , 'poly_dust_service')
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
        if response.photo != array('B'):
            # Convertir el objeto array a una matriz NumPy del tipo uint8
            reconstructed_bytes = bytes(response.photo)
            reconstructed_np_array = np.frombuffer(reconstructed_bytes, dtype=np.uint8)

            image_array = np.reshape(reconstructed_np_array, _image_size)
        
            image = cv2.resize(image_array,(_image_size[1]//2, _image_size[0]//2))
            cv2.imshow("poly_dust_client",  image )
            cv2.waitKey(0)
    finally:
        minimal_client.destroy_node()
        rclpy.shutdown()
        # cv2.destroyAllWindows()

if __name__ == '__main__':
    main()