


from adafruit_servokit import ServoKit

ARM_CHANNEL = 2
HEAD_CHANNEL = 3

ARM_MIN = 10
ARM_MAX = 52


def get_motor_output_from_cmd(x, z):
    # Calculate speeds for each motor
    speed_A = x + z
    speed_B = x - z

    # Normalize and map to angle values
    pwm_a = max(0, min(180, (speed_A / 2) * 180))  # Motor A forward
    pwm_b = max(0, min(180, (-speed_A / 2) * 180)) # Motor A backward
    pwm_c = max(0, min(180, (speed_B / 2) * 180))  # Motor B forward
    pwm_d = max(0, min(180, (-speed_B / 2) * 180)) # Motor B backward

    # map
    return pwm_a, pwm_b, pwm_c, pwm_d


class AngelaMovement:
    def __init__(self, head_idx, left_arm_idx, right_arm_idx, motor_a_fwd_idx, motor_a_bwd_idx, motor_b_fwd_idx , motor_b_bwd_idx): 
        self.board  = ServoKit(channels=16, frequency = 60)

        # servos
        self.left_arm = self.board.servo[left_arm_idx]
        self.right_arm = self.board.servo[right_arm_idx]
        self.head = self.board.servo[head_idx]

        #motors
        self.left_motor_fwd = self.board.servo[motor_a_fwd_idx]
        self.left_motor_bwd = self.board.servo[motor_a_bwd_idx]
        self.right_motor_fwd = self.board.servo[motor_b_fwd_idx]
        self.right_motor_bwd = self.board.servo[motor_b_bwd_idx]
        
    def move(self,linearVel, angularVel):

        angles = get_motor_output_from_cmd(linearVel, angularVel)
        self.left_motor_fwd.angle = angles[0]
        self.left_motor_bwd.angle = angles[1]
        self.right_motor_fwd.angle = angles[2]
        self.right_motor_bwd.angle = angles[3]
