import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray


class JointPublisher(Node):
    def __init__(self):
        super().__init__('joint_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.subscription = self.create_subscription(Float32MultiArray, 'pos_data', self.pos_callback, 10)
        self.js0 = JointState()
        self.pos = [0.0, 0.0, 0.0, 0.0]
        self.js0.name = ['body_to_top', 'right_to_center', 'center_to_arm2', 'arm2_to_arm3']
        self.tmr = self.create_timer(0.05, self.callback)

    def pos_callback(self, pos_msg):
        self.pos[0] = pos_msg.data[0]
        self.pos[1] = pos_msg.data[1]
        self.pos[2] = pos_msg.data[2]
        self.pos[3] = pos_msg.data[3]

    def callback(self):
        self.js0.header.stamp = self.get_clock().now().to_msg()
        self.js0.position = self.pos
        self.publisher_.publish(self.js0)


def main():
    rclpy.init()
    joint_publisher = JointPublisher()
    rclpy.spin(joint_publisher)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

