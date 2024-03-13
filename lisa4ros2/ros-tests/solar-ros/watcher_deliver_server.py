from solar_interfaces.srv import DeliverImg

import rclpy
from rclpy.node import Node

from .redis_store import get_uint8


class DeliverService(Node):

    def __init__(self):
        super().__init__('deliver_service')
        self.srv = self.create_service(DeliverImg, 'deliver_server', self.deliver_photo)

    def deliver_photo(self, request, response):
        self.get_logger().info(f'Incoming request photo with id {request.photo_id}')
        ret, image = get_uint8(str(request.photo_id))
        
        response.photo = []
        if ret:
            response.photo = [int(byte) for byte in image]
        self.get_logger().info(f'served photo with id {request.photo_id}')
        return response


def main():
    rclpy.init()

    deliver_service = DeliverService()

    rclpy.spin(deliver_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()