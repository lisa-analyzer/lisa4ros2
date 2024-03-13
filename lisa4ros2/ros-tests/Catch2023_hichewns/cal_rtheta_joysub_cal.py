import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray


class JoySubscriber(Node):
    def __init__(self):
        super().__init__('joy_subscriber')
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.publisher_ = self.create_publisher(Float32MultiArray, 'joy_data', 10)
        self.tmr = self.create_timer(0.1, self.callback)
        self.data = [0.0, 0.0, 0.0, 0.0]

    def joy_callback(self, joy_msg):
        axes = joy_msg.axes
        buttons = joy_msg.buttons


        self.data[0] = axes[0]  # revolute (top_linkの回転)左軸左右回転
        self.data[1] = axes[1]  # prismatic (center_linkの直動)右軸上下
        self.data[2] = axes[6]
        self.data[3] = axes[7]


    def callback(self):
        joy_data = Float32MultiArray()
        joy_data.data = self.data
        self.publisher_.publish(joy_data)


def main():
    rclpy.init()
    joy_subscriber = JoySubscriber()
    rclpy.spin(joy_subscriber)
    rclpy.shutdown()


if __name__ == '__main__':
    main()


