import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from catch2023_interfaces.msg import CreateMessage
import math
import time

MAX_R = 0.48

class XY_to_Rtheta(Node):
    def __init__(self):
        #publisher
        super().__init__('xy_to_rtheta')
        self.degpos_publisher = self.create_publisher(CreateMessage, 'degpos_data', 10)
        self.pos_publisher = self.create_publisher(Float32MultiArray, 'pos_data', 10)
        # self.flag_publisher = self.create_publisher(String, 'is_ended', 10)
        #subscriber
        self.index_subscription = self.create_subscription(Int8, 'index', self.index_callback, 10)
        self.shooting_index_subscription = self.create_subscription(Int8, 'shooting_index', self.shooting_index_callback, 10)
        self.cmd_state_subscription = self.create_subscription(String,'cmd_state',self.cmd_state_callback,10)
        self.shooting_error_publisher = self.create_publisher(Float32, 'shooting_error', 10)
        self.target_error_publisher = self.create_publisher(Float32, 'target_error', 10)
        self.joy_subscription = self.create_subscription(Float32MultiArray, 'joy_data', self.joy_callback, 10)
        self.stepper_subscription = self.create_subscription(Int32, 'stepper_cmd', self.stepper_callback,10)
        self.target_subscription = self.create_subscription(Float32MultiArray, 'target_xy',self.target_callback,  10)
        self.shootingbox_subscripition = self.create_subscription(Float32MultiArray, 'shootingbox_xy', self.shooting_callback,10)
        self.release_subscription = self.create_subscription(Bool, 'release_cmd', self.release_callback, 10)
        self.cur_subscription = self.create_subscription(CreateMessage, 'real_pos', self.real_pos_callback, 10)
        self.move_subscription = self.create_subscription(Bool, 'move_cmd', self.move_callback, 10)
        self.init_subscription = self.create_subscription(Bool, 'init', self.init_callback, 10)
        self.target_pose_publisher = self.create_publisher(Float32MultiArray, 'target_pose', 10)
        self.shooting_pose_publisher = self.create_publisher(Float32MultiArray, 'shooting_pose', 10)
        self.target_comp_publisher = self.create_publisher(Bool, 'target_comp', 10)
        self.target_comp_r_publisher = self.create_publisher(Bool, 'target_comp_r', 10)
        
        self.start_subscription = self.create_subscription(Bool, 'start', self.start_callback, 10)
        self.shooting_comp_publisher = self.create_publisher(Bool, 'shooting_comp', 10)
        # self.joint_subscription = self.create_subscription(JointState, 'joint_states', self.joint_states_callback,10)
        
        self.tmr = self.create_timer(0.1, self.callback)
        self.catch = 0.0
        self.release = 0.0
        self.init = 0.0
        self.start = False
        self.currentPos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.degPos = [0.0, 0.0, 0.0, 0.0, 0.0, [0,0,0]]
        self.cur_x = 0.0
        self.cur_y = 0.0
        self.target_x = 0.0
        self.target_y = 0.0
        self.shooting_x = 0.0
        self.shooting_y = 0.0


        self.STP_INIT_POS = 0 #0
        self.STP_SHOOT_POS = 40 #1
        self.STP_POI_POS = 110 #1.5
        self.STP_SERCH_POS = 205 #2
        self.STP_COMMON_POS = 220 #3
        self.STP_PUT_POS = 250 #4
        self.STP_OWN_POS = 270 #5
        self.stepper_pos = self.STP_INIT_POS

        self.servo_cmd = 1
        self.real_r = 0.0
        self.real_theta = 0.0
        self.shooting_error = 0.0
        self.shooting_error_r = 0.0
        self.target_error = 0.0
        self.target_error_r = 0.0

        self.rev = 0.0
        self.degrev = 0.0
        self.rev_shooting = 0.0
        self.degrev_shooting = 0.0
        self.index_prev = 0
        self.shooting_index_prev = 0
        
        self.ERROR_theta = 45.0
        self.ERROR_target_theta = 120.0
        self.target_error_th = 120.0
        self.flag = 0

        self.target_comp = False
        self.shooting_comp = False
        self.target_comp_r = False

        self.theta = 0.0
        self.y = 0.0
        self.armtheta = 0.0
        # arm2
        self.up = 0.0
        self.mid = 0.0
        self.down = 0.0
        self.revarm3 = 0.0
        self.flag = 1
        self.rev = 0.0 #hand1-3

        self.is_change = False

    def start_callback(self,start_msg):
        self.start = start_msg.data
        if self.start == True:
            self.currentPos = [0.0, 0.0, 0.08, 0.0, 0.0, 0.0, 0.0]
            self.degPos = [0.0, 0.0, 0.0, 0.0, 0.0, [0,0,0]]

    def init_callback(self,init_msg):
        self.init = init_msg.data
        if self.init == True:
            self.currentPos = [math.pi/2, 0.0, 0.08, 0.0, 0.0, 0.0, 0.0]
            self.degPos = [90.0, 0.0, 0.0, 0.0, 0.0, [1,1,1]]
    
    def real_pos_callback(self,real_pos_msg):
        self.real_theta = real_pos_msg.theta
        self.real_r = real_pos_msg.r 
        self.real_armtheta = real_pos_msg.armtheta
    
    def release_callback(self,release_msg):
        if release_msg.data == True:
            if self.is_change == False:
                self.degPos[5] = [0,0,0]
        
        elif release_msg.data == False:
            if self.is_change == False:
                self.degPos[5] = [1,1,1]
    
    def index_callback(self,index_msg):
        self.index = index_msg.data
        if self.index == 0:
            self.shooting_index = 6
        
    def shooting_index_callback(self,shooting_index_msg):
        self.shooting_index = shooting_index_msg.data

    def cmd_state_callback(self,cmd_state_msg):
        if cmd_state_msg.data == 'c':
            # self.degPos[5] = True
            self.degPos[5] = [1,1,1]
            self.is_change = False
        
        elif cmd_state_msg.data == 'r':
            # self.degPos[5] = False
            self.degPos[5] = [0,0,0]
            self.is_change = False
        
        if cmd_state_msg.data == 'c0':
            self.degPos[5][0] = 1
            self.is_change = True
        
        elif cmd_state_msg.data == 'r0':
            self.degPos[5][0] = 0
            self.is_change = True
        
        if cmd_state_msg.data == 'c1':
            self.degPos[5][1] = 1
            self.is_change = True
        
        elif cmd_state_msg.data == 'r1':
            self.degPos[5][1] = 0
            self.is_change = True
        
        if cmd_state_msg.data == 'c2':
            self.degPos[5][2] = 1
            self.is_change = True
        
        elif cmd_state_msg.data == 'r2':
            self.degPos[5][2] = 0
            self.is_change = True
            
    def move_callback(self,move_msg):
        self.move_cmd = move_msg.data
        if self.move_cmd == True:
            if self.degPos[1] > 0.157:
                self.currentPos[1] = 0.157
                if self.currentPos[1] >= MAX_R:
                    self.currentPos[1] = MAX_R
                elif self.currentPos[1] <= 0.0:
                    self.currentPos[1] = 0.0

                self.degPos[1] = 0.157
                if self.degPos[1] >= MAX_R:
                    self.degPos[1] = MAX_R
                elif self.degPos[1] <= 0.0:
                    self.degPos[1] = 0.0

    def joy_callback(self, joy_msg):
        self.theta = joy_msg.data[0]
        self.y = joy_msg.data[1]
        self.armtheta = joy_msg.data[5]

            
    def target_callback(self, target_msg):
        self.target_error_r = 0.0
        self.target_comp = False
        self.target_x = target_msg.data[0]
        self.target_y = target_msg.data[1]
        self.target_pose = [self.target_x,self.target_y]

        self.currentPos[0]=math.atan2(self.target_y,self.target_x)
        self.degPos[0]=math.degrees(math.atan2(self.target_y,self.target_x))

        self.target_error = float(self.degPos[0] - self.real_theta)   
        self.target_error = abs(self.target_error)   

        self.target_error_r = float(self.degPos[1] - (self.real_r/1000))
        self.target_error_r = abs(self.target_error_r)

        # if self.servo_cmd == 1:
        if not self.index == self.index_prev:
            self.rev = 0.0
            self.degrev = 0.0
    
        if self.index < 6:
            self.currentPos[4]=-math.pi/4
            self.currentPos[5]=-math.pi/4
            self.currentPos[6]=-math.pi/4
            self.degPos[4] = -45.0

            self.currentPos[3]=((math.pi / 2) - self.currentPos[0]) + math.pi/4 + self.rev
            self.degPos[3] = (90 - self.degPos[0]) + 45 + self.degrev
            if self.degPos[3] < 0:
                self.degPos[3] += 180.0
            self.index_prev = self.index

        elif self.index >= 6:
        # elif self.servo_cmd == 0:
            self.currentPos[4]=0.0
            self.currentPos[5]=0.0
            self.currentPos[6]=0.0
            self.degPos[4] = 0.0

            self.currentPos[3]= -self.currentPos[0] + self.rev
            self.degPos[3] = -self.degPos[0] + self.degrev
            if self.degPos[3] < 0:
                self.degPos[3] += 180.0
            self.index_prev = self.index
        
        if self.armtheta > 0:
            self.rev += self.armtheta / 50
            self.degrev += math.degrees(self.armtheta / 50)
            if self.degrev > 180:
                self.degrev = 180
            
        elif self.armtheta < 0:
            self.rev -= abs(self.armtheta) / 50
            self.degrev -= math.degrees(abs(self.armtheta) / 50)
            if self.degrev < 0:
                self.degrev = 0

        if self.target_error <= self.ERROR_target_theta:
            self.currentPos[1]=math.sqrt(self.target_x**2+self.target_y**2) - 0.407

            self.degPos[1]=math.sqrt(self.target_x**2+self.target_y**2) - 0.407
            
            #Joyの計算部分
            if self.theta >= 0:
                self.currentPos[0] +=  self.theta/ 100
                self.degPos[0]  += math.degrees(self.theta / 100)

            elif self.theta < 0:
                self.currentPos[0] -= abs(self.theta) / 100
                self.degPos[0] -= math.degrees(abs(self.theta) / 100)

            if self.y >= 0:
                self.currentPos[1] += self.y / 500
                self.degPos[1] += self.y / 500

            elif self.y < 0:
                self.currentPos[1] -= abs(self.y) / 500
                self.degPos[1] -= abs(self.y) / 500

            # ここまで
            self.target_pose[0] = math.cos(self.currentPos[0])* (self.degPos[1]+0.407)
            self.target_pose[1] = math.sin(self.currentPos[0])* (self.degPos[1]+0.407)
            targetpose = Float32MultiArray()
            targetpose.data = self.target_pose
            self.target_pose_publisher.publish(targetpose)
            # if self.target_error_r <= 0.05 and self.y == 0.0 and self.theta == 0.0:
            # and self.y == 0.0 and self.theta == 0.0:

            if self.index == 1:
                self.target_error_th = 60.0
            elif not self.index == 1 and self.index < 6:
                self.target_error_th = 120.0
            
            elif self.index >= 6:
                self.target_error_th = 60.0
  
            if self.target_error < self.target_error_th:
                self.target_comp  = True
            
            if self.target_error < 0.2:
                self.target_comp_r = True
            # elif self.target_error_r > 0.02:
            #     self.target_comp = False

        targeterror = Float32()
        targeterror.data = self.target_error
        self.target_error_publisher.publish(targeterror)
            
        targetcomp = Bool()
        targetcomp.data = self.target_comp
        self.target_comp_publisher.publish(targetcomp)

        targetcomp_r = Bool()
        targetcomp_r.data = self.target_comp_r
        self.target_comp_r_publisher.publish(targetcomp_r)
        
        # elif self.target_error > self.ERROR_theta:
        #     self.target_comp = False
        # self.target_comp=False

    
    def shooting_callback(self, shooting_msg):
        self.shooting_error_r = 0.0
        self.shooting_x = shooting_msg.data[0]
        self.shooting_y = shooting_msg.data[1]
        self.shooting_pose = [self.shooting_x,self.shooting_y]
        self.currentPos[0] = math.atan2(self.shooting_y,self.shooting_x)
        self.degPos[0] = math.degrees(math.atan2(self.shooting_y,self.shooting_x))

        self.shooting_error = float(self.degPos[0] - self.real_theta)
        self.shooting_error = abs(self.shooting_error)
        if self.shooting_error <= self.ERROR_theta:
            self.currentPos[1] = math.sqrt(self.shooting_x**2 +self.shooting_y**2) - 0.407
            self.degPos[1] = math.sqrt(self.shooting_x**2+self.shooting_y**2) - 0.407
                            #Joyの計算部分
            if self.theta > 0:
                self.currentPos[0] +=  self.theta/ 100
                self.degPos[0]  += math.degrees(self.theta / 100)

            elif self.theta < 0:
                self.currentPos[0] -= abs(self.theta) / 100
                self.degPos[0] -= math.degrees(abs(self.theta) / 100)

            if self.y > 0:
                self.currentPos[1] += self.y / 500
                self.degPos[1] += self.y / 500

            elif self.y < 0:
                self.currentPos[1] -= abs(self.y) / 500
                self.degPos[1] -= abs(self.y) / 500
            
            # if self.armtheta > 0:
            #     self.currentPos[3] += self.armtheta / 100
            #     self.degPos[3] += self.armtheta / 100
            
            # elif self.armtheta < 0:
            #     self.currentPos[3] -= abs(self.armtheta) / 100
            #     self.degPos[3] -= abs(self.armtheta) / 100

            # ここまで
            self.shooting_pose[0] = math.cos(self.currentPos[0])* (self.degPos[1]+0.407)
            self.shooting_pose[1] = math.sin(self.currentPos[0])* (self.degPos[1]+0.407)
            shootingpose = Float32MultiArray()
            shootingpose.data = self.shooting_pose
            self.shooting_pose_publisher.publish(shootingpose)

            self.shooting_error_r = float(self.degPos[1] - (self.real_r/1000))
            self.shooting_error_r = abs(self.shooting_error_r)
            if self.shooting_error_r <= 0.01:
            # and self.y == 0.0 and self.theta == 0.0:
                self.shooting_comp = True
            elif self.shooting_error_r > 0.01:
                self.shooting_comp = False
            #     self.degPos[5] = False

            shooting_comp = Bool()
            shooting_comp.data = self.shooting_comp
            self.shooting_comp_publisher.publish(shooting_comp)
        
            shootingerror = Float32()
            shootingerror.data = self.shooting_error_r
            self.shooting_error_publisher.publish(shootingerror)

        if not self.shooting_index == self.shooting_index_prev:
            self.rev_shooting = 0.0
            self.degrev_shooting = 0.0
        
        if self.shooting_index == 7:
            self.currentPos[4]=0.0
            self.currentPos[5]=0.0
            self.currentPos[6]=0.0
            self.degPos[4] = 0.0
            self.currentPos[3]= -self.currentPos[0]+self.rev_shooting
            self.degPos[3] = -self.degPos[0] + self.degrev_shooting
            if self.degPos[3] < 0:
                self.degPos[3] += 180.0
            self.shooting_index_prev = self.shooting_index
        
        elif not self.shooting_index == 7:
            self.currentPos[3]=-(self.currentPos[0] - math.pi/2) + self.rev_shooting
            self.currentPos[4]=0.0
            self.currentPos[5]=0.0
            self.currentPos[6]=0.0

            self.degPos[3]=-(self.degPos[0] - 90.0) + self.degrev_shooting
            if self.degPos[3] < 0:
                self.degPos[3] += 180.0
            self.degPos[4]=0.0
            self.shooting_index_prev = self.shooting_index


        if self.armtheta > 0:
            self.rev_shooting += self.armtheta / 50
            self.degrev_shooting += math.degrees(self.armtheta / 50)
            if self.degrev_shooting > 180:
                self.degrev_shooting = 180.0
            
            
        elif self.armtheta < 0:
            self.rev_shooting -= abs(self.armtheta) / 50
            self.degrev_shooting -= math.degrees(abs(self.armtheta) / 50)
            if self.degrev_shooting < 0:
                self.degrev_shooting = 0.0

    
    def stepper_callback(self,msg):
        self.stepper_pos = msg.data
        self.degPos[2]= self.stepper_pos

        if self.stepper_pos == self.STP_INIT_POS:
            self.currentPos[2] = 0.08
        
        if self.stepper_pos == self.STP_POI_POS:
            self.currentPos[2] = 0.07
        
        if self.stepper_pos == self.STP_SHOOT_POS:
            self.currentPos[2] = 0.06
        
        if self.stepper_pos == self.STP_PUT_POS:
            self.currentPos[2] = 0.04
        
        if self.stepper_pos == self.STP_COMMON_POS:
            self.currentPos[2] = -0.04
        
        if self.stepper_pos == self.STP_PUT_POS:
            self.currentPos[2] = -0.06
        
        if self.stepper_pos == self.STP_OWN_POS:
            self.currentPos[2] = -0.08
            self.currentPos[2] = -0.08

    
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
        # degpos_data.judge = bool(self.degPos[5])
        degpos_data.hand_state = self.degPos[5]
        self.degpos_publisher.publish(degpos_data)

def main():
    rclpy.init()
    xy_to_rtheta = XY_to_Rtheta()
    rclpy.spin(xy_to_rtheta)
    rclpy.shutdown()

if __name__ == '__main__':
    main()