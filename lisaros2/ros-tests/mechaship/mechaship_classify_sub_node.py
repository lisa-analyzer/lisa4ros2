import math

import cv2
import numpy as np
import rclpy
from mechaship_interfaces.msg import ClassificationArray
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data


class MechashipClassifySub(Node):
    def __init__(self):
        super().__init__(
            "mechaship_classify_sub_node",
            allow_undeclared_parameters=  True,
            automatically_declare_parameters_from_overrides=True,
        )
        self.map_size = (
            self.get_parameter_or(
                "map_size", Parameter("map_size", Parameter.Type.INTEGER, 100)
            )
            .get_parameter_value()
            .integer_value
        )
        self.turn_degree = (
            self.get_parameter_or(
                "turn_degree", Parameter("turn_degree", Parameter.Type.INTEGER, 270)
            )
            .get_parameter_value()
            .integer_value
        )
        self.map_scale = (
            self.get_parameter_or(
                "map_scale", Parameter("map_scale", Parameter.Type.INTEGER, 100)
            )
            .get_parameter_value()
            .integer_value
        )

        self.get_logger().info("map_size: %s" % (str(self.map_size)))
        self.get_logger().info("turn_degree: %s" % (str(self.turn_degree)))
        self.get_logger().info("map_scale: %s" % (str(self.map_scale)))

        self.buoy_subscription = self.create_subscription(
            ClassificationArray,
            "/Buoy",
            self.buoy_listener_callback,
            qos_profile_sensor_data,
        )
        self.wall_subscription = self.create_subscription(
            ClassificationArray,
            "/Wall",
            self.wall_listener_callback,
            qos_profile_sensor_data,
        )

        self.buoy_subscription  # prevent unused variable warning
        self.wall_subscription  # prevent unused variable warning

        self.buoy = ClassificationArray()
        self.wall = ClassificationArray()

    def wall_listener_callback(self, data):
        self.get_logger().info("wall cnt: %s" % (len(data.classifications)))
        self.wall = data

    def buoy_listener_callback(self, data):
        self.get_logger().info("buoy cnt: %s" % (len(data.classifications)))
        self.buoy = data

        self.show_classification_array(self.buoy, "buoy", (100, 100), True)
        self.show_classification_array(self.wall, "wall", (100, 100), True)

        self.show_map((self.map_size, self.turn_degree, self.map_scale))

        cv2.waitKey(1)

    def show_classification_array(
        self,
        classification_array,
        title="classifications",
        scale=(100, 100),
        iscolor=True,
    ):
        colors = [
            (0, 0, 255),
            (0, 255, 0),
            (255, 0, 0),
            (0, 255, 255),
            (255, 0, 255),
            (255, 255, 0),
        ]

        height = int(3.5 * scale[0])
        width = int(2 * math.pi * scale[1])
        empty_image = np.zeros((height, width, 3), np.uint8)  # (세로, 가로)
        for idx, classification in enumerate(classification_array.classifications):
            if iscolor:
                color = colors[idx % (len(colors))]
            else:
                color = (255, 255, 255)
            for range, theta in zip(classification.ranges, classification.thetas):
                empty_image[int(range * scale[0])][int(theta * scale[1])] = color

        cv2.imshow(title, np.flip(empty_image, axis=0))

    def show_map(self, scale=(100, 270, 100)):
        # scale = (map_size, turn_degree, scale)
        image_size = int(7 * scale[0])
        self.get_logger().info("image_size: %s" % (image_size))

        turn_radians = math.radians(scale[1])
        center_point = (int(image_size / 2), int(image_size / 2))
        self.get_logger().info("center_point: %s" % (str(center_point)))
        empty_image = np.zeros((image_size, image_size, 3), np.uint8)  # (세로, 가로)
        cv2.line(empty_image, center_point, center_point, (255, 0, 0), 3)
        for classification in self.buoy.classifications:
            for range, theta in zip(classification.ranges, classification.thetas):
                point_x = int(
                    math.cos(-theta + turn_radians) * range * scale[2] + center_point[0]
                )
                point_y = int(
                    math.sin(-theta + turn_radians) * range * scale[2] + center_point[1]
                )
                cv2.line(
                    empty_image, (point_x, point_y), (point_x, point_y), (0, 0, 255), 2
                )

        for classification in self.wall.classifications:
            for range, theta in zip(classification.ranges, classification.thetas):
                point_x = int(
                    math.cos(-theta + turn_radians) * range * scale[2] + center_point[0]
                )
                point_y = int(
                    math.sin(-theta + turn_radians) * range * scale[2] + center_point[1]
                )
                cv2.line(
                    empty_image, (point_x, point_y), (point_x, point_y), (255, 0, 0), 2
                )

        cv2.imshow("cartesian_coordinates", empty_image)


def main(args=None):
    rclpy.init(args=args)
    node = MechashipClassifySub()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
