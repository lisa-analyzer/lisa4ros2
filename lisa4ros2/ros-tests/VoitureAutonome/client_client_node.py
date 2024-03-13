import select
import time
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Bool
import socket


class ClientNode(Node):

    def __init__(self):
        super().__init__('client_node')
        timer_period = 0.5  # seconds
        self.isRaceOn = False
        self.publisher_connexion = self.create_publisher(Bool, 'IsClientConnected', 1)
        #continiously post the connexion state
        self.PostConnexion = self.create_timer(timer_period, self.PostConnexionState)


        self.publisher_race = self.create_publisher(Bool, 'IsClientRaceOn', 1)
        self.PostIsRaceOn = self.create_timer(timer_period, self.PostIsRaceOn)

        self.subscriber_race = self.create_subscription(Bool, 'IsAppRaceOn', self.getMessageRace, 1)

    def getMessageRace(self, message: Bool):
        self.isRaceOn = message.data

    def PostIsRaceOn(self):
        if(type(self.isRaceOn) == bool):
            msg = Bool()
            msg.data = self.isRaceOn
            self.publisher_race.publish(msg)

    def PostConnexionState(self):
        msg = Bool()
        msg.data = True
        self.publisher_connexion.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    client_node = ClientNode()

    rclpy.spin(client_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    client_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()