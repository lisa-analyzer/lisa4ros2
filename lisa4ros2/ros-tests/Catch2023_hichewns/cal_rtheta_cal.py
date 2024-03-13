import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class Cal(Node):
    def __init__(self):
        super().__init__('cal')
        self.pos_publisher = self.create_publisher(Float32MultiArray, 'pos_data', 10)
        self.subscription = self.create_subscription(Float32MultiArray, 'joy_data', self.joy_callback, 10)
        self.tmr = self.create_timer(0.1, self.callback)
        self.r = 0.0
        self.y = 0.0
        self.up = 0
        self.down = 0
        self.rev = 0.0
        self.currentPos = [0.0, 0.0, 0.0, 0.0]

    def joy_callback(self, joy_msg):
        self.r = joy_msg.data[0]
        self.y = joy_msg.data[1]
        self.up = int(joy_msg.data[2])
        self.rev = int(joy_msg.data[3])

        if self.r >= 0:
            self.currentPos[0] += self.r / 100
        else:
            self.currentPos[0] -= abs(self.r) / 100
        if self.y > 0:
            self.currentPos[1] += self.y / 100
            if self.currentPos[1]>=0.6:
                self.currentPos[1]=0.6
        else:
            self.currentPos[1] -= abs(self.y) / 100
            if self.currentPos[1]<=0.25:
                self.currentPos[1]=0.25

        if self.up == 1:
            self.currentPos[2] += 0.04
            if self.currentPos[2]>=0.04:
                self.currentPos[2]=0.04
        elif self.up == -1:
            self.currentPos[2] -= 0.08
            if self.currentPos[2]<=-0.04:
                self.currentPos[2]=-0.04
                

        if self.rev == 1:
            self.currentPos[3] += 1.0
        elif self.rev == -1:
            self.currentPos[3] -= 1.0
        

    def callback(self):
        pos_data = Float32MultiArray()
        pos_data.data = self.currentPos
        self.pos_publisher.publish(pos_data)


def main():
    rclpy.init()
    cal = Cal()
    rclpy.spin(cal)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

