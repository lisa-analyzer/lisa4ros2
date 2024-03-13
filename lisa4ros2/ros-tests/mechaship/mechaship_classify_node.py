import math

import numpy as np
import rclpy
from mechaship_interfaces.msg import Classification, ClassificationArray
from numpy.polynomial import Polynomial
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class MechashipClassify(Node):
    def __init__(self) -> None:
        super().__init__(
            "mechaship_classify_node",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        self.classify_method = (
            self.get_parameter_or(
                "classify_method",
                Parameter("classify_method", Parameter.Type.STRING, "lsm"),
            )
            .get_parameter_value()
            .string_value
        )
        self.differential_threshold = (
            self.get_parameter_or(
                "differential_threshold",
                Parameter("differential_threshold", Parameter.Type.DOUBLE, 5.0),
            )
            .get_parameter_value()
            .double_value
        )
        self.max_rho_index_range = (
            self.get_parameter_or(
                "max_rho_index_range",
                Parameter("max_rho_index_range", Parameter.Type.DOUBLE, 0.0),
            )
            .get_parameter_value()
            .double_value
        )
        self.classify_min_length = (
            self.get_parameter_or(
                "classify_min_length",
                Parameter("classify_min_length", Parameter.Type.INTEGER, 10),
            )
            .get_parameter_value()
            .integer_value
        )
        self.classify_max_length = (
            self.get_parameter_or(
                "classify_max_length",
                Parameter("classify_max_length", Parameter.Type.INTEGER, 100),
            )
            .get_parameter_value()
            .integer_value
        )
        self.classify_index_range = (
            self.get_parameter_or(
                "classify_index_range",
                Parameter("classify_index_range", Parameter.Type.DOUBLE, 0.3),
            )
            .get_parameter_value()
            .double_value
        )

        self.get_logger().info("classify_method: %s" % (self.classify_method))
        self.get_logger().info(
            "differential_threshold: %s" % (self.differential_threshold)
        )
        self.get_logger().info("max_rho_index_range: %s" % (self.max_rho_index_range))
        self.get_logger().info("classify_min_length: %s" % (self.classify_min_length))
        self.get_logger().info("classify_max_length: %s" % (self.classify_max_length))
        self.get_logger().info("classify_index_range: %s" % (self.classify_index_range))

        self.scan_subscription = self.create_subscription(
            LaserScan, "/scan", self.listener_callback, qos_profile_sensor_data
        )
        self.scan_subscription  # prevent unused variable warning

        self.buoy_publisher = self.create_publisher(
            ClassificationArray, "Buoy", qos_profile_sensor_data
        )
        self.wall_publisher = self.create_publisher(
            ClassificationArray, "Wall", qos_profile_sensor_data
        )

    def listener_callback(self, data: LaserScan) -> None:
        """/scan 토픽 콜백

        Args:
            data (LaserScan): 라이다 데이터(/scan)
        """
        self.get_logger().info("ranges cnt: %s" % (len(data.ranges)))
        self.get_logger().info("rad min: %s" % (math.degrees(data.angle_min)))
        self.get_logger().info("rad max: %s" % (math.degrees(data.angle_max)))
        self.get_logger().info("range min: %s" % (data.range_min))
        self.get_logger().info("range max: %s" % (data.range_max))

        rhos = []  # 거리(M)
        thetas = []  # 각도(radian)
        angle = data.angle_min
        for scan_data in data.ranges:
            if (scan_data != 0) and (not math.isinf(scan_data)):
                rhos.append(scan_data)
                thetas.append(angle)
            angle += data.angle_increment

        # 극좌표계 미분해서 기울기로 그룹핑
        polar_coordinates_group = self.grouping_with_differential(
            rhos, thetas, self.differential_threshold
        )

        if self.classify_method == "rho":
            # max rho로 분리하는 방법
            polar_coordinates_group = self.regrouping_with_max_rho(
                polar_coordinates_group, self.max_rho_index_range
            )
            buoy, wall = self.classify_with_rho(
                polar_coordinates_group,
                self.classify_min_length,
                self.classify_index_range,
            )
        elif self.classify_method == "lsm":
            # 최소차승법으로 분리하는 방법
            buoy, wall = self.classify_with_LSM(
                polar_coordinates_group,
                self.classify_min_length,
                self.classify_max_length,
            )
        else:
            return

        self.buoy_publisher.publish(buoy)
        self.wall_publisher.publish(wall)

    def grouping_with_differential(
        self, rhos: list, thetas: list, threshold: float
    ) -> list:
        """극좌표계 좌표(각도, 거리)를 미분해서 기울기의 임계값으로 그룹핑

        Args:
            rhos (list): 거리
            thetas (list): 각도
            threshold (float): 기울기 임계값

        Returns:
            list: 그룹핑된 극좌표계 2차원 배열(rho, theta)
        """
        rhos_df = np.r_[0, np.diff(rhos)] / (thetas[1] - thetas[0])

        polar_coordinates_group = []
        polar_coordinates = []
        for rho, theta, rho_df in zip(rhos, thetas, rhos_df):
            if abs(rho_df) > threshold:
                polar_coordinates_group.append(polar_coordinates)
                polar_coordinates = []

            polar_coordinates.append((rho, theta))

        return polar_coordinates_group

    def regrouping_with_max_rho(
        self, polar_coordinates_group: list, index_range: float = 0
    ) -> list:
        """한 그룹의 최대 거리(max_rho)를 기준으로 재그룹핑
        (최대 거리의 index가 처음 또는 끝이 아닐 경우 그룹 분리)

        Args:
            polar_coordinates_group (list): 그룹핑된 극좌표계 2차원 배열(rho, theta)
            index_range (float, optional): 그룹핑 옵션(0~1). Defaults to 0.

        Returns:
            list: 재그룹핑된 극좌표계 2차원 배열(rho, theta)
        """
        polar_coordinates_regroup = []
        for polar_coordinates in polar_coordinates_group[2:]:
            rhos = [x[0] for x in polar_coordinates]
            max_rho_index = rhos.index(max(rhos))

            if index_range == 0:
                if max_rho_index != 0 and max_rho_index != len(polar_coordinates) - 1:
                    polar_coordinates_regroup.append(polar_coordinates[:max_rho_index])
                    polar_coordinates_regroup.append(polar_coordinates[max_rho_index:])
                else:
                    polar_coordinates_regroup.append(polar_coordinates)

            elif index_range < 1:
                min_index = (len(polar_coordinates) - 1) * index_range
                max_index = (len(polar_coordinates) - 1) * (1 - index_range)
                if max_rho_index > min_index and max_rho_index < max_index:
                    polar_coordinates_regroup.append(polar_coordinates[:max_rho_index])
                    polar_coordinates_regroup.append(polar_coordinates[max_rho_index:])
                else:
                    polar_coordinates_regroup.append(polar_coordinates)

        return polar_coordinates_regroup

    def classify_with_rho(
        self, polar_coordinates_group: list, min_length: int = 0, index_range: float = 0
    ) -> tuple:
        """rho로 부표와 벽으로 분류(min_rho가 중간, max_rho가 처음 또는 끝일 경우)

        Args:
            polar_coordinates_group (list): 그룹핑된 극좌표계 2차원 배열(rho, theta)
            min_length (int, optional): 한 그룹 내 최소 좌표 개수. Defaults to 0.
                (min_length보다 좌표 개수가 적은 그룹은 생략)
            index_range (float, optional): 그룹핑 옵션(0~1). Defaults to 0.

        Returns:
            tuple: 부표와 벽으로 구분된 Classification 메세지
        """
        buoy = Classification()
        wall = Classification()

        for polar_coordinates in polar_coordinates_group:
            length = len(polar_coordinates)
            if length < min_length:
                continue

            rhos = [x[0] for x in polar_coordinates]
            min_rho_index = rhos.index(min(rhos))
            if min_rho_index == 0 or min_rho_index == length - 1:
                wall.ranges.append(polar_coordinates[0])
                wall.thetas.append(polar_coordinates[1])
                continue

            left_max_rho = max(rhos[:min_rho_index])
            right_max_rho = max(rhos[min_rho_index:])
            if abs(left_max_rho - right_max_rho) > 0.07:
                wall.ranges.append(polar_coordinates[0])
                wall.thetas.append(polar_coordinates[1])
                continue

            if index_range == 0:
                if min_rho_index == int(len(polar_coordinates) * 0.5 - 1):
                    buoy.ranges.append(polar_coordinates[0])
                    buoy.thetas.append(polar_coordinates[1])
                else:
                    wall.ranges.append(polar_coordinates[0])
                    wall.thetas.append(polar_coordinates[1])

            elif index_range < 1:
                min_index = (len(polar_coordinates) - 1) * (0.5 - index_range)
                max_index = (len(polar_coordinates) - 1) * (0.5 + index_range)
                if min_index < min_rho_index < max_index:
                    buoy.ranges.append(polar_coordinates[0])
                    buoy.thetas.append(polar_coordinates[1])
                else:
                    wall.ranges.append(polar_coordinates[0])
                    wall.thetas.append(polar_coordinates[1])

            elif index_range >= 1:
                min_index = len(polar_coordinates) * 0.5 - 1 - index_range
                max_index = len(polar_coordinates) * 0.5 - 1 + index_range
                if min_index < min_rho_index < max_index:
                    buoy.ranges.append(polar_coordinates[0])
                    buoy.thetas.append(polar_coordinates[1])
                else:
                    wall.ranges.append(polar_coordinates[0])
                    wall.thetas.append(polar_coordinates[1])

        return buoy, wall

    def classify_with_LSM(
        self, polar_coordinates_group: list, min_length: int = 0, max_length: int = 100
    ) -> tuple:
        """최소차승법으로 부표와 벽으로 분류(기울기가 양인 경우 부표로 분리)

        Args:
            polar_coordinates_group (list): 그룹핑된 극좌표계 2차원 배열(rho, theta)
            min_length (int, optional): 한 그룹 내 최소 좌표 개수. Defaults to 0.
                (min_length보다 좌표 개수가 적은 그룹은 생략)
            max_length (int, optional): 한 그룹 내 최대 좌표 개수. Defaults to 100.
                (max_length보다 좌표 개수가 많은 그룹은 생략)

        Returns:
            tuple: 부표와 벽으로 구분된 Classification 메세지
        """
        buoy = ClassificationArray()
        wall = ClassificationArray()

        for polar_coordinates in polar_coordinates_group:
            length = len(polar_coordinates)
            if length < min_length or length > max_length:
                continue

            coords = Classification()

            coords.ranges = [x[0] for x in polar_coordinates]
            coords.thetas = [x[1] for x in polar_coordinates]

            # 최소 차승법
            fx = Polynomial.fit(coords.thetas, coords.ranges, deg=2).convert()
            axis = fx.coef[1] / (-2.0 * fx.coef[2])
            self.get_logger().info("fx: %s" % (str(fx)))
            self.get_logger().info("axis: %s" % (str(axis)))

            center_theta = coords.thetas[int(len(coords.thetas) / 2.0)]
            if fx.coef[2] > 0 and center_theta * 0.9 < axis < center_theta * 1.1:
                buoy.classifications.append(coords)
            else:
                wall.classifications.append(coords)

        return buoy, wall


def main(args=None):
    rclpy.init(args=args)
    node = MechashipClassify()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
