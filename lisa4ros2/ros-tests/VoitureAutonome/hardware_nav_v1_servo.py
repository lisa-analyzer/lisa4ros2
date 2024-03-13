import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from std_msgs.msg import Float32
import time 

# Define input and output ranges as constants
INPUT_MIN = -0.27
INPUT_MAX = 0.27
OUTPUT_MIN = 2.5
OUTPUT_MAX = 6.6
PWM_FREQUENCY = 50
NEUTRAL_POS = (OUTPUT_MIN+OUTPUT_MAX)/2

class Servo(Node):

    def __init__(self):
        super().__init__('servo')

        self.subscriber_connexion = self.create_subscription(Float32, 'servoSteer', self.getServoSteer, 10)

        #GPIO 13  for the steering control
        self.pwm_pin = 13
        GPIO.setmode(GPIO.BCM) #BCM mode
        GPIO.setup(self.pwm_pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pwm_pin, PWM_FREQUENCY)

        #init sequence
        self.pwm.start(NEUTRAL_POS)
        #self.pwm.ChangeDutyCycle(steer)
   
    def map_value(self, value, in_min, in_max, out_min, out_max):
        map =(value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
        rounded_value = round(map, 1)
        return rounded_value
    
    def getServoSteer(self, message: Float32):
        servo_converted = self.map_value(message.data, INPUT_MIN, INPUT_MAX, OUTPUT_MIN, OUTPUT_MAX)
        #self.get_logger().info("servo_converted:" + str(servo_converted))
        self.pwm.ChangeDutyCycle(servo_converted)

def main(args=None):
    rclpy.init(args=args)

    servo = Servo()

    rclpy.spin(servo)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    servo.destroy_node()
    rclpy.shutdown()
    #GPIO.cleanup()

if __name__ == '__main__':
    main()
