import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from catch2023_interfaces.msg import CreateMessage
import math
import time

class Joy_operation(Node):
    def __init__(self):
        super().__init__('joy_operation')
        self.pos_publisher = self.create_publisher(Float32MultiArray, 'pos_data', 10)
        # self.catch_judge_publisher = self.create_publisher(Bool, 'catch_data', 10)
        # self.degpos_publisher = self.create_publisher(Float32MultiArray, 'degpos_data', 10)
        self.degpos_publisher = self.create_publisher(CreateMessage, 'degpos_data', 10)
        self.subscription = self.create_subscription(Float32MultiArray, 'joy_data', self.joy_callback, 10)
        self.tmr = self.create_timer(0.1, self.callback)
        self.status = 0
        self.theta = 0.0
        self.y = 0.0
        # arm2
        self.up = 0.0
        self.mid = 0.0
        self.down = 0.0
        self.revarm3 = 0.0
        self.flag = 1
        self.rev = 0.0 #hand1-3
        self.catch = 0.0
        self.release = 0.0
        self.init = 0.0
        self.grasp = Bool()
        self.currentPos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.degPos = [0.0, 0.0, 0.0, 0.0, 0.0, [0,0,0]]

    def joy_callback(self, joy_msg):
        self.theta = joy_msg.data[0]
        self.y = joy_msg.data[1]
        self.mid = joy_msg.data[2]
        self.up = joy_msg.data[3]
        self.down = joy_msg.data[4]
        self.revarm3 = joy_msg.data[5]
        self.rev = joy_msg.data[6]
        self.catch = joy_msg.data[7]
        self.release = joy_msg.data[8]

        if self.theta >= 0:
            self.currentPos[0] += self.theta/ 100
            self.degPos[0] += math.degrees(self.theta / 100)
            if self.currentPos[0]>=2*math.pi:
                self.currentPos[0]=2*math.pi
            if self.degPos[0]>=360.0:
                self.degPos[0]=360.0

        else:
            self.currentPos[0] -= abs(self.theta) / 100
            self.degPos[0] -= math.degrees(abs(self.theta) / 100)
            if self.currentPos[0]<=0.0:
                self.currentPos[0]=0.0
            # if self.degPos[0]<=0.0:
            #     self.degPos[0]=0.0

        if self.y > 0:
            self.currentPos[1] += self.y / 500
            self.degPos[1] += self.y / 500
            if self.currentPos[1]>=0.542:
                self.currentPos[1]=0.542
            if self.degPos[1]>=0.542:
                self.degPos[1]=0.542

        else:
            self.currentPos[1] -= abs(self.y) / 500
            self.degPos[1] -= abs(self.y) / 500
            if self.currentPos[1]<=0.0:
                self.currentPos[1]=0.0
            if self.degPos[1]<=0.0:
                self.degPos[1]=0.0

        if self.up == 1.0:
            self.currentPos[2] = 0.08
            self.degPos[2]= 0
            # time.sleep(0.1)
        
        if self.mid == 1.0:
            self.currentPos[2] = 0.04
            self.degPos[2] = 1
            # time.sleep(0.1)
        
        if self.down == 1.0:
            self.currentPos[2] = -0.08
            self.degPos[2] = 2
            # time.sleep(0.1)

                
        if self.revarm3 >= 0.0:
            self.currentPos[3] += self.revarm3 / 50
            self.degPos[3] += math.degrees(self.revarm3 / 50)
            if self.currentPos[3]>=math.pi:
                self.currentPos[3]=math.pi
            if self.degPos[3]>=180.0:
                self.degPos[3]=180.0

        else:
            self.currentPos[3] -= abs(self.revarm3) / 50
            self.degPos[3] -= math.degrees(abs(self.revarm3) / 50)
            if self.currentPos[3]<=0.0:
                self.currentPos[3]=0.0
            if self.degPos[3]<=0.0:
                self.degPos[3]=0.0

        if self.catch==1.0:
            self.grasp.data = True
            self.degPos[5] = [1,1,1]
        
        if self.release==1.0:
            self.grasp.data = False
            self.degPos[5] = [0,0,0]

        # self.catch_judge_publisher.publish(self.grasp)
        
        if self.rev == 1.0:
            if self.flag == 1:
                if self.status==1:
                    self.currentPos[4] = math.pi/4
                    self.currentPos[5] = math.pi/4
                    self.currentPos[6] = math.pi/4
                    self.degPos[4] = math.degrees(math.pi/4)
                    self.status = 0
                    self.flag = 0

                elif self.status==0:
                    self.currentPos[4] = 0.0
                    self.currentPos[5] = 0.0
                    self.currentPos[6] = 0.0
                    self.degPos[4] = math.degrees(0.0)
                    self.status = 1
                    self.flag = 0 

            elif self.flag == 0:
                time.sleep(0.2)
                self.flag = 1

    def callback(self):
        pos_data = Float32MultiArray()
        pos_data.data = self.currentPos
        self.pos_publisher.publish(pos_data)

        degpos_data = CreateMessage()
        degpos_data.theta = self.degPos[0]
        degpos_data.r = self.degPos[1]
        degpos_data.r *= 1000
        degpos_data.stepper = int(self.degPos[2])
        degpos_data.armtheta = self.degPos[3]
        degpos_data.hand= self.degPos[4]
        degpos_data.hand_state = self.degPos[5]
        # degpos_data.data = self.degPos
        # degpos_data.data[1]*=1000
        self.degpos_publisher.publish(degpos_data)


def main():
    rclpy.init()
    cal = Joy_operation()
    rclpy.spin(cal)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
