#!/usr/bin/env -S HOME=${HOME} ${HOME}/.virtualenvs/cv/bin/python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String,UInt8


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_1 = self.create_publisher(String, '/chessboard/pseudo_fen', 10)
        self.publisher_2= self.create_publisher(UInt8, '/chessboard/AI_ready', 10)
        self.publisher_3 = self.create_publisher(String, '/chessboard/AI_bestmove', 10)

        timer_period = 0.5  # seconds
        self.timer1 = self.create_timer(timer_period, self.timer_callback1)
        self.i = 0
        timer_period = 0.5  # seconds
        self.timer2 = self.create_timer(timer_period, self.timer_callback2)
        self.i = 0
        timer_period = 0.5  # seconds
        self.timer3 = self.create_timer(timer_period, self.timer_callback3)
        self.i = 0

    def timer_callback1(self):
        msg = String()
        msg.data = 'rnbqkbnr/p1pppp1p/1P6/5Pp1/8/8/1PPPP1PP/RNBQKBNR'
        self.publisher_1.publish(msg)
        self.get_logger().info('Publishing: FEN')
    def timer_callback2(self):
        msg = UInt8()
        if(self.i%30 == 0):
            msg.data = 2
        else:
            msg.data = 0
        self.publisher_2.publish(msg)
        self.get_logger().info('Publishing: READY')
        self.i = 0
    def timer_callback3(self):
        msg = String()
        msg.data = "f5g6"
        self.publisher_3.publish(msg)
        self.get_logger().info('Publishing: MOVE')


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()