import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import time

INPUT_MIN = 0
INPUT_MAX = 20
ANGLE_MIN = 90
ANGLE_MAX = 160
PWM_FREQUENCY = 50
NEUTRAL_POS = 89

class Motor(Node):

    def __init__(self):
        super().__init__('motor')
        self.subscriber_motor_speed = self.create_subscription(Float32, 'motorSpeed', self.getMotorSpeed, 1)

        self.pwm_pin = 12
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pwm_pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pwm_pin, PWM_FREQUENCY)
        GPIO.output(12, True)
        self.pwm.start(0)
        self.previousValue = 86

    def map_to_angle_to_pwm(self, value):
        m = (ANGLE_MAX - ANGLE_MIN) / (INPUT_MAX - INPUT_MIN)
        b = ANGLE_MAX - (INPUT_MAX * m)
        angle_value = m * value + b

        if(value < 0):
            angle_value = 86
            if self.previousValue != angle_value:
                self.set_pwm(angle_value)
        elif(value == 0):
            angle_value = 89
            self.set_pwm(angle_value)
        else:
            angle_value = m * value + b
            angle_value = max(100, angle_value)
            angle_value = 100
            self.set_pwm(angle_value)

        self.previousValue = angle_value
        return angle_value

    def set_pwm(self, angle):
        duty_cycle = (angle / 18) + 2
        self.pwm.ChangeDutyCycle(duty_cycle)
        self.get_logger().info(f"speed duty cycle angle: {angle}")
    
    def getMotorSpeed(self, message: Float32):
        self.map_to_angle_to_pwm(message.data)

def main(args=None):
    rclpy.init(args=args)

    motor = Motor()

    rclpy.spin(motor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motor.destroy_node()
    rclpy.shutdown()

    #GPIO.cleanup()

if __name__ == '__main__':
    main()
