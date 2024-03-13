import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class CommunicationNode(Node):

    def __init__(self):
        super().__init__('communication_node')
        self.isConnected = False
        self.isRaceOn = False
        self.timeOutCounter = 0
        timer_period = 1  # seconds
        self.publisher_connexion = self.create_publisher(Bool, 'IsConnected', 1)
        #continiously post the connexion state
        self.PostConnexion = self.create_timer(timer_period, self.PostConnexionState)


        self.publisher_race = self.create_publisher(Bool, 'IsRaceOn', 1)
        self.PostIsRaceOn = self.create_timer(timer_period, self.PostIsRaceOn)

        #continiously call IsConnected() at the timer_period rate
        self.ConnexionTimer = self.create_timer(timer_period, self.IsConnected)


        self.subscriber_race = self.create_subscription(Bool, 'IsClientRaceOn', self.getMessageRace, 1)
        self.subscriber_connexion = self.create_subscription(Bool, 'IsClientConnected', self.getMessageConnexion, 1)

    def getMessageRace(self, message: Bool):
        self.isRaceOn = message.data

    def getMessageConnexion(self, message: Bool):
        if(message.data):
            self.isConnected = True
            self.timeOutCounter = 0

    def PostIsRaceOn(self):
        msg = Bool()
        msg.data = self.isRaceOn
        self.publisher_race.publish(msg)

    def PostConnexionState(self):
        msg = Bool()
        msg.data = self.isConnected
        self.publisher_connexion.publish(msg)

    def IsConnected(self):
        self.timeOutCounter += 1
        if (self.timeOutCounter <= 10):
            self.isConnected = False


def main(args=None):
    rclpy.init(args=args)

    communication_node = CommunicationNode()

    rclpy.spin(communication_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    communication_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()