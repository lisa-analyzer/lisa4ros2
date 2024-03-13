import os
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from std_msgs.msg import String


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('index')

    publisher = node.create_publisher(Int8, 'index', 10)

    msg = Int8()

    while rclpy.ok():
        index = int(input('Enter Index: '))
        msg.data = index
        node.get_logger().info('Publishing: "%s"' % msg)
        publisher.publish(msg)


if __name__ == '__main__':
    main() 
