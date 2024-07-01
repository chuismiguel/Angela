import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
import neopixel
import board

GPIO_MOTOR1_1 = 27
GPIO_MOTOR1_2 = 22
GPIO_MOTOR2_1 = 23
GPIO_MOTOR2_2 = 24

class MotorController(Node):

    def __init__(self):
        super().__init__('motor_controller')
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(GPIO_MOTOR1_1, GPIO.OUT)
        self.mot1 = GPIO.PWM(GPIO_MOTOR1_1, 50)  # Here 50 is the frequency of 50Hz
        self.mot1.start(0)

        GPIO.setup(GPIO_MOTOR1_2, GPIO.OUT)
        self.mot2 = GPIO.PWM(GPIO_MOTOR1_2, 50)
        self.mot2.start(0)

        GPIO.setup(GPIO_MOTOR2_1, GPIO.OUT)
        self.mot3 = GPIO.PWM(GPIO_MOTOR2_1, 50)
        self.mot3.start(0)

        GPIO.setup(GPIO_MOTOR2_2, GPIO.OUT)
        self.mot4 = GPIO.PWM(GPIO_MOTOR2_2, 50)
        self.mot4.start(0)

        self.pixels = neopixel.NeoPixel(board.D13, 6)

        # Subscribe to cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        self.subscription  # prevent unused variable warning

    def cmd_vel_callback(self, msg):
        self.get_logger().info(f'Received cmd_vel: linear.x={msg.linear.x}, angular.z={msg.angular.z}')

        linear_x = msg.linear.x
        angular_z = msg.angular.z
        self.pixels.fill((0,255,0))
        
        # Forward or backward
        if linear_x > 0:
            self.mot1.start(0)
            self.mot2.start(50)
            self.mot3.start(50)
            self.mot4.start(0)
        elif linear_x < 0:
            self.mot1.start(50)
            self.mot2.start(0)
            self.mot3.start(0)
            self.mot4.start(50)
        else:
            self.mot1.start(0)
            self.mot2.start(0)
            self.mot3.start(0)
            self.mot4.start(0)
        # Turning
        if angular_z > 0:
            self.mot1.start(50)
            self.mot2.start(0)
            self.mot3.start(50)
            self.mot4.start(0)
        elif angular_z < 0:
            self.mot1.start(0)
            self.mot2.start(50)
            self.mot3.start(0)
            self.mot4.start(50)
        else:
            self.mot1.start(0)
            self.mot2.start(0)
            self.mot3.start(0)
            self.mot4.start(0)

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    rclpy.spin(motor_controller)

    # Destroy the node explicitly
    motor_controller.destroy_node()
    rclpy.shutdown()
    GPIO.cleanup()

if __name__ == '__main__':
    main()