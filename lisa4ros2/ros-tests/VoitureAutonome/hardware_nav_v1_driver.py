import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from std_msgs.msg import Bool
from interfaces.msg import Sensors
from interfaces.action import DriverAction
from interfaces.msg import DrivingInstruction
from sensor_msgs.msg import LaserScan
from serial import Serial
import json
from interfaces.msg import MonAckermannDrive
import time

import pyudev

class Driver(Node):

    def __init__(self):
        super().__init__('driver')

        self.publisher_servo = self.create_publisher(Float32, 'servoSteer', 1)
        self.publisher_motor = self.create_publisher(Float32, 'motorSpeed', 1)

        self.subscriber_lidar = self.create_subscription(LaserScan, 'LidarScanPreprocessed', self.__cmd_scan_callback, 1)

        timer_period = 0.01
        #self.PostConnexion = self.create_timer(timer_period, self.dummy)

        # self._action_client = ActionClient(self, DriverAction, 'driverAction')
        # self._current_goal_future = None

        #self.get_logger().info(str(self._get_arduino_udev()))
        self.ser = Serial(self._get_arduino_udev(), 115200, timeout=0, rtscts=True)
        self.previousData = {"servoSteer": 45, "motorSpeed": 90}

        self.publisher_lidar_brain = self.create_publisher(LaserScan, 'LidarBrain', 1)
        self.subscriber_driving_command = self.create_subscription(MonAckermannDrive, 'BrainDrivingCommand', self.send_driving_command, 1)

    def dummy(self):
        self.__cmd_scan_callback(LaserScan())

    def _get_arduino_udev(self):
        context = pyudev.Context() #2341:0043 #'03eb', '2145'
        return find_single_device('2341', '0043',context) #TODO: mettre à jour ces valeurs avec celle de la carte arduino correct

    def __cmd_scan_callback(self, message):

        self.publisher_lidar_brain.publish(message)

    def send_driving_command(self,message):
        motorSpeed = Float32()
        servoSteer = Float32()
        motorSpeed.data = message.speed
        servoSteer.data = message.steering_angle
        self.publisher_motor.publish(motorSpeed)
        self.publisher_servo.publish(servoSteer)

        data = {"servoSteer": self.map_servo(servoSteer.data), "motorSpeed": self.map_speed(motorSpeed.data)}
        if data != self.previousData:
            data_str = json.dumps(data)  # conversion en chaîne de caractères JSON
            self.get_logger().info(str(data_str))
            try:
                self.ser.write(data_str.encode())  # envoi des données encodées en bytes
                self.ser.write("\n".encode())
            except:
                time.sleep(0.5)
                self.get_logger().info("Crash")
                self.ser = Serial(self._get_arduino_udev(), 115200, timeout=0, xonxoff=True)
                self.ser.write(data_str.encode())  # envoi des données encodées en bytes
                self.ser.write("\n".encode())

            self.previousData = data
            

    def map_speed(self, value):
        INPUT_MIN = -20
        INPUT_MAX = 20
        OUTPUT_MIN = 10
        OUTPUT_MAX = 170

        m = (OUTPUT_MAX - OUTPUT_MIN) / (INPUT_MAX - INPUT_MIN)
        b = OUTPUT_MAX - (INPUT_MAX * m)

        if(value < 0):
            angle_value = 78
        elif(value == 0):
            angle_value = 90
        else:
            angle_value = m * value + b
            angle_value = min(180, angle_value)
        return int(angle_value)
    
    def map_servo(self, value):
        INPUT_MIN = -0.27
        INPUT_MAX = 0.27
        OUTPUT_MIN = 25
        OUTPUT_MAX = 65

        m = (OUTPUT_MAX - OUTPUT_MIN) / (INPUT_MAX - INPUT_MIN)
        b = OUTPUT_MAX - (INPUT_MAX * m)

        angle_value = m * value + b
        return int(angle_value)

def device_by_id(id_vendor, id_product,context):
    match_devices = []
    for dev in context.list_devices(subsystem='tty'):
            
        try:
            idVendor = dev.properties['ID_VENDOR_ID']
            idProduct = dev.properties['ID_MODEL_ID']
        except :
                continue
        if idVendor == id_vendor and idProduct == id_product:
            match_devices.append(dev)
    return match_devices


def find_single_device(id_vendor, id_product,context):
    devs = device_by_id(id_vendor, id_product, context)
    if len(devs) == 0:
        msg = "Unable to find device with idVendor={} and idProduct={}"
        raise ValueError(msg.format(id_vendor, id_product))
    elif len(devs) > 1:
        msg = "Found multiple devices with idVendor={} and idProduct={}"
        raise ValueError(msg.format(id_vendor, id_product))
    else:
        return devs[0].device_node

def main(args=None):
    rclpy.init(args=args)

    driver = Driver()

    rclpy.spin(driver)

    # while True:
    #     rclpy.spin_once(driver.__node, timeout_sec=0)
    #     if driver._current_goal_future is not None:
    #         rclpy.spin_until_future_complete(driver.__node, driver._current_goal_future)
    #         driver._current_goal_future = None

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
