import cv2
import rclpy
from cv_bridge import CvBridge
from mechaship_interfaces.msg import DetectionArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class MechashipDetectSub(Node):
    def __init__(self):
        super().__init__(  "mechaship_detect_sub_node")

        self.image_subscription = self.create_subscription(
            Image, "/Image", self.image_listener_callback, qos_profile_sensor_data
        )
        self.detection_subscription = self.create_subscription(
            DetectionArray,
            "/DetectionArray",
            self.detection_listener_callback,
            qos_profile_sensor_data,
        )

        self.image_subscription  # prevent unused variable warning
        self.detection_subscription  # prevent unused variable warning

        self.br = CvBridge()
        self.detections = DetectionArray().detections

    def detection_listener_callback(self, data):
        self.get_logger().info("detection cnt: %s" % (len(data.detections)))
        self.detections = data.detections

    def image_listener_callback(self, data):
        origin_image = self.br.imgmsg_to_cv2(data, "bgr8")
        if origin_image.all() == None or len(origin_image) == 0:
            return

        if len(self.detections) != 0:
            for detection in self.detections:
                cv2.rectangle(
                    origin_image,
                    (int(detection.xmin), int(detection.ymin)),
                    (int(detection.xmax), int(detection.ymax)),
                    (0, 0, 255),
                    3,
                )
        cv2.imshow("detected image", origin_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = MechashipDetectSub()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
