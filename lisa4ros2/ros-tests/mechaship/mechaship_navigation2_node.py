import math

import rclpy
from mechaship_interfaces.msg import DetectionArray
from mechaship_interfaces.srv import Key, RGBColor, ThrottlePercentage
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class MechashipNavigation2(Node):
    def __init__(self) -> None:
        super().__init__(
            "mechaship_navigation2_node",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        self.range_distance = (
            self.get_parameter_or(
                "range_distance",
                Parameter("range_distance", Parameter.Type.DOUBLE, 1.0),
            )
            .get_parameter_value()
            .double_value
        )
        self.range_start_angle = (
            self.get_parameter_or(
                "range_start_angle",
                Parameter("range_start_angle", Parameter.Type.INTEGER, 85),
            )
            .get_parameter_value()
            .integer_value
        )
        self.range_end_angle = (
            self.get_parameter_or(
                "range_end_angle",
                Parameter("range_end_angle", Parameter.Type.INTEGER, 95),
            )
            .get_parameter_value()
            .integer_value
        )
        self.docking_shape = (
            self.get_parameter_or(
                "docking_shape",
                Parameter("docking_shape", Parameter.Type.STRING, "blue_circle"),
            )
            .get_parameter_value()
            .string_value
        )
        self.waypoint_right = (
            self.get_parameter_or(
                "waypoint_right",
                Parameter("waypoint_right", Parameter.Type.STRING, "vertical_buoy"),
            )
            .get_parameter_value()
            .string_value
        )
        self.waypoint_left = (
            self.get_parameter_or(
                "waypoint_left",
                Parameter("waypoint_left", Parameter.Type.STRING, "horizontal_buoy"),
            )
            .get_parameter_value()
            .string_value
        )
        self.docking_color_r = (
            self.get_parameter_or(
                "docking_color_r",
                Parameter("docking_color_r", Parameter.Type.INTEGER, 0),
            )
            .get_parameter_value()
            .integer_value
        )
        self.docking_color_g = (
            self.get_parameter_or(
                "docking_color_g",
                Parameter("docking_color_g", Parameter.Type.INTEGER, 0),
            )
            .get_parameter_value()
            .integer_value
        )
        self.docking_color_b = (
            self.get_parameter_or(
                "docking_color_b",
                Parameter("docking_color_b", Parameter.Type.INTEGER, 255),
            )
            .get_parameter_value()
            .integer_value
        )

        self.get_logger().info("range_distance: %s" % (str(self.range_distance)))
        self.get_logger().info("range_start_angle: %s" % (str(self.range_start_angle)))
        self.get_logger().info("range_end_angle: %s" % (str(self.range_end_angle)))
        self.get_logger().info("docking_shape: %s" % (str(self.docking_shape)))
        self.get_logger().info("docking_color_r: %s" % (str(self.docking_color_r)))
        self.get_logger().info("docking_color_g: %s" % (str(self.docking_color_g)))
        self.get_logger().info("docking_color_b: %s" % (str(self.docking_color_b)))

        self.scan_subscription = self.create_subscription(
            LaserScan, "/scan", self.scan_listener_callback, qos_profile_sensor_data
        )

        self.detection_subscription = self.create_subscription(
            DetectionArray,
            "/DetectionArray",
            self.detection_listener_callback,
            qos_profile_sensor_data,
        )

        self.scan_subscription  # prevent unused variable warning
        self.detection_subscription  # prevent unused variable warning

        self.create_timer(0.5, self.navigate)

        self.set_key_handler = self.create_client(Key, "/actuators/key/set")
        self.set_throttle_handler = self.create_client(
            ThrottlePercentage, "/actuators/throttle/set_percentage"
        )
        self.set_color_handler = self.create_client(RGBColor, "set_color")

        self.targets = {
            "ranges": [],
            "waypoint": {"left": 0, "right": 0},
            "docking": [],
        }
        self.dangerous_angles = []
        self.camera_fov = 62.2
        self.side_angle = (180 - self.camera_fov) / 2.0
        self.image_width = 400

    def scan_listener_callback(self, data: LaserScan) -> None:
        """/scan 토픽 콜백

        Args:
            data (LaserScan): 라이다 데이터(/scan)
        """
        # self.get_logger().info("ranges cnt: %s" % (len(data.ranges)))
        # self.get_logger().info("rad min: %s" % (math.degrees(data.angle_min)))
        # self.get_logger().info("rad max: %s" % (math.degrees(data.angle_max)))
        # self.get_logger().info("range min: %s" % (data.range_min))
        # self.get_logger().info("range max: %s" % (data.range_max))

        target_ranges = []
        dangerous_angles = []
        angle = data.angle_min
        for scan_data in data.ranges:
            if scan_data != 0 or not math.isinf(scan_data):
                angle += data.angle_increment
                continue

            if scan_data < self.range_distance:
                dangerous_angles.append(round(math.degrees(angle)))

            if angle > math.radians(self.range_end_angle):
                break
            elif angle > math.radians(self.range_start_angle):
                target_ranges.append(scan_data)
            angle += data.angle_increment

        self.targets["ranges"] = target_ranges
        self.dangerous_angles = dangerous_angles

    def detection_listener_callback(self, data: DetectionArray) -> None:
        """/DetectionArray 토픽 콜백

        Args:
            data (DetectionArray): 객체 탐지 데이터(/DetectionArray)
        """
        # self.get_logger().info("detection cnt: %s" % (len(data.detections)))
        waypoint = [0, 0]
        self.targets["docking"] = []
        for detection in data.detections:
            if detection.name == self.waypoint_left:
                waypoint[0] = detection
            elif detection.name == self.waypoint_right:
                waypoint[1] = detection
            elif detection.name == self.docking_shape:
                color = RGBColor.Request()
                color.red = self.docking_color_r
                color.green = self.docking_color_g
                color.blue = self.docking_color_b
                self.set_color_handler.call_async(color)
                self.targets["docking"] = [detection]
        self.targets["waypoint"]["left"] = waypoint[0]
        self.targets["waypoint"]["right"] = waypoint[1]

    def constrain(self, input_vel: float, low_bound: float, high_bound: float) -> float:
        """지정된 범위 내의 값으로 반환

        Args:
            input_vel (float): 입력값
            low_bound (float): 최소값
            high_bound (float): 최대값

        Returns:
            float: 반환값
        """
        if input_vel < low_bound:
            input_vel = low_bound
        elif input_vel > high_bound:
            input_vel = high_bound
        else:
            input_vel = input_vel

        return input_vel

    def navigate(self) -> None:
        """주행 알고리즘"""
        key = Key.Request()
        throttle = ThrottlePercentage.Request()

        # range_distance 이내에 장애물이 있을 경우(감속, 회피)
        if len(self.dangerous_angles) > 0:
            self.get_logger().info("dangerous")
            dangerous_angles_mean = round(
                sum(self.dangerous_angles) / len(self.dangerous_angles)
            )
            goal_angle = (360 + 90 - dangerous_angles_mean) % 360
            key.degree = self.constrain(goal_angle, 60, 120)
            throttle.percentage = 15

        # 경유할 경우
        elif (
            self.targets["waypoint"]["left"] != 0
            or self.targets["waypoint"]["right"] != 0
        ):
            self.get_logger().info("waypoint")
            waypoint_left = self.targets["waypoint"]["left"]
            waypoint_right = self.targets["waypoint"]["right"]
            if waypoint_left == 0:  # 좌측 부표 X(우측 부표만 인식)
                goal_angle = 180 - int(
                    (waypoint_right.xmin * self.camera_fov) / self.image_width
                    + self.side_angle
                )
                key.degree = self.constrain(goal_angle + 10, 60, 120)
            elif waypoint_right == 0:  # 우측 부표 X(좌측 부표만 인식)
                goal_angle = 180 - int(
                    (waypoint_left.xmax * self.camera_fov) / self.image_width
                    + self.side_angle
                )
                key.degree = self.constrain(goal_angle - 10, 60, 120)
            else:  # 부표 둘 다 인식
                center_x = (waypoint_left.xmax + waypoint_right.xmin) / 2.0
                goal_angle = 180 - int(
                    (center_x * self.camera_fov) / self.image_width + self.side_angle
                )
                key.degree = self.constrain(goal_angle, 60, 120)
            throttle.percentage = 20

        # docking 할 경우
        elif len(self.targets["docking"]) > 0:
            self.get_logger().info("docking")
            docking_target = self.targets["docking"][0]
            center_x = (docking_target.xmin + docking_target.xmax) / 2.0
            goal_angle = 180 - int(
                (center_x * self.camera_fov) / self.image_width + self.side_angle
            )
            key.degree = self.constrain(goal_angle, 60, 120)
            throttle.percentage = 20

        # 우측 벽을 따라 운항
        else:
            self.get_logger().info("go")
            key.degree = 95
            throttle.percentage = 20

        self.set_key_handler.call_async(key)
        self.set_throttle_handler.call_async(throttle)


def main(args=None):
    rclpy.init(args=args)
    node = MechashipNavigation2()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
