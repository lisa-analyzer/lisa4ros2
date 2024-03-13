import cv2
import json
import pkg_resources

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from solar_tools.ImageReader import take_pick_from_stream
from solar_tools.Model import  Yolo5


from .redis_store import send_to_cache


class WactherPublisher(Node):

    def __init__(self):
        super().__init__('watcher_publisher')
        self.publisher_ = self.create_publisher(String, 'solar_panel_watcher', 10)
        model_path = pkg_resources.resource_filename('watcher', 'models/bestV3.pt')
        self.yolo5 = Yolo5(model_path)
        timer_period = 0.3  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        # leemos la imagen del stream 
        image = take_pick_from_stream()
        # la guardamos en el cache
        if send_to_cache(self.i, image):
            # si se guardo la procesamos por el modelo
            results = self.yolo5.get_image_markers_over_confidence(image, 0.7)
            # Mandamos los resultados al publisher
            post = {'id' : self.i, 'results' : results}
            msg.data = json.dumps(post, default=str)
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
            self.i += 1


def main(args=None):
    rclpy.init(args=args)

    watcher_publisher = WactherPublisher()

    rclpy.spin(watcher_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    watcher_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()