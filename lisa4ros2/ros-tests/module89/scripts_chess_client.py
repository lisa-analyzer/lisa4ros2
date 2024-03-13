#!/usr/bin/env -S HOME=${HOME} ${HOME}/.virtualenvs/cv/bin/python

from module89.srv import FindBestMove       # CHANGE
from std_msgs.msg import String
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('chess_client_async')
        self.cli = self.create_client(FindBestMove, 'find_best_move')       # CHANGE
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = FindBestMove.Request()                                   # CHANGE

    def send_request(self):
        fen = String()
        fen.data = "8/2p4k/2q1pQ1p/4pN2/3br3/7P/5Pr1/5R1K b - - 1 2"
        self.req.fen = fen
        self.future = self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)

    chess_client = MinimalClientAsync()
    chess_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(chess_client)
        if chess_client.future.done():
            try:
                response = chess_client.future.result()
            except Exception as e:
                chess_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                chess_client.get_logger().info(
                    'Best move of FEN %s is %s' %                               # CHANGE
                    (chess_client.req.fen.data, response.bestmove.data)) # CHANGE
            break

    chess_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()