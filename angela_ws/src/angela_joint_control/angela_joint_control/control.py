#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from adafruit_servokit import ServoKit

RIGHT_ARM_CHANNEL = 4  
LEFT_ARM_CHANNEL = 3
HEAD_CHANNEL = 5

ARM_MIN = 20
ARM_MAX = 75

HEAD_MIN = 0
HEAD_MAX = 180

servo_board = ServoKit(channels=16, frequency=60)

class Arm:
    def __init__(self): 
        self.rightArm = servo_board.servo[RIGHT_ARM_CHANNEL]
        self.leftArm = servo_board.servo[LEFT_ARM_CHANNEL]
        
    def position(self, angle):
        if angle < ARM_MIN:
            angle = ARM_MIN
        elif angle > ARM_MAX:
            angle = ARM_MAX

        self.rightArm.angle = (180 - angle)
        self.leftArm.angle = angle
        
class Head:
    def __init__(self):
        self.head = servo_board.servo[HEAD_CHANNEL]
        
    def position(self, angle):
        if angle < HEAD_MIN:
            angle = HEAD_MIN
        elif angle > HEAD_MAX:
            angle = HEAD_MAX
        self.head.angle = (180 - angle)

class ServoControlNode(Node):
    def __init__(self):
        super().__init__('servo_control_node')
        self.subscription = self.create_subscription(
            JointState,
            'angela/joint_states',
            self.listener_callback,
            10)
        self.arm = Arm()
        self.head = Head()
    
    def listener_callback(self, msg):
        for name, position in zip(msg.name, msg.position):
            if name == 'head':
                self.head.position(position)
            elif name == 'arm':
                self.arm.position(position)

def main(args=None):
    rclpy.init(args=args)
    node = ServoControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
