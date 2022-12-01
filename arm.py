''' handle arm 
translated from arm.c and arm.h
https://github.com/cyberbotics/webots/blob/master/projects/robots/kuka/youbot/libraries/youbot_control/src/arm.c
'''

from controller import Robot, Motor
from math import pi, sqrt, asin, acos, atan

robot = Robot()

class Arm:
    def __init__(self, robot):
        self.robot = robot

        self.height = {
            'ARM_FRONT_FLOOR': 0, 'ARM_FRONT_PLATE': 1, 'ARM_HANOI_PREPARE': 2, 'ARM_FRONT_CARDBOARD_BOX': 3,
            'ARM_RESET': 4, 'ARM_BACK_PLATE_HIGH': 5, 'ARM_BACK_PLATE_LOW': 6, 'ARM_MAX_HEIGHT': 7
            }

        self.orientation = {
            'ARM_BACK_LEFT': 0, 'ARM_LEFT': 1, 'ARM_FRONT_LEFT': 2, 'ARM_FRONT': 3,
            'ARM_FRONT_RIGHT': 4, 'ARM_RIGHT': 5, 'ARM_BACK_RIGHT': 6, 'ARM_MAX_SIDE': 7
            }

        self.current_height = self.height['ARM_RESET']
        self.current_orientation = self.orientation['ARM_FRONT']

        self.arm1 = self.robot.getDevice('arm1')
        self.arm2 = self.robot.getDevice('arm2')
        self.arm3 = self.robot.getDevice('arm3')
        self.arm4 = self.robot.getDevice('arm4')
        self.arm5 = self.robot.getDevice('arm5')

        self.arm2.setVelocity(0.5)

        self.arm_set_height(self.height['ARM_RESET'])
        self.arm_set_orientation(self.orientation['ARM_FRONT'])

    def arm_reset(self):
        self.arm1.setPosition(0.0)
        self.arm2.setPosition(1.57)
        self.arm3.setPosition(-2.635)
        self.arm4.setPosition(1.78)
        self.arm5.setPosition(0.0)
    
    def arm_set_height(self, ht):
        if ht == self.height['ARM_FRONT_FLOOR']:
            self.arm2.setPosition(-0.97)
            self.arm3.setPosition(-1.55)
            self.arm4.setPosition(-0.61)
            self.arm5.setPosition(0.0)
        elif ht == self.height['ARM_FRONT_PLATE']:
            self.arm2.setPosition(-0.62)
            self.arm3.setPosition(-0.98)
            self.arm4.setPosition(-1.53)
            self.arm5.setPosition(0.0)
        elif ht == self.height['ARM_FRONT_CARDBOARD_BOX']:
            self.arm2.setPosition(0.0)
            self.arm3.setPosition(-0.77)
            self.arm4.setPosition(-1.21)
            self.arm5.setPosition(0.0)
        elif ht == self.height['ARM_RESET']:
            self.arm2.setPosition(1.57)
            self.arm3.setPosition(-2.635)
            self.arm4.setPosition(1.78)
            self.arm5.setPosition(0.0)
        elif ht == self.height['ARM_BACK_PLATE_HIGH']:
            self.arm2.setPosition(0.678)
            self.arm3.setPosition(0.682)
            self.arm4.setPosition(1.74)
            self.arm5.setPosition(0.0)
        elif ht == self.height['ARM_BACK_PLATE_LOW']:
            self.arm2.setPosition(0.92)
            self.arm3.setPosition(0.42)
            self.arm4.setPosition(1.78)
            self.arm5.setPosition(0.0)
        elif ht == self.height['ARM_HANOI_PREPARE']:
            self.arm2.setPosition(-0.4)
            self.arm3.setPosition(-1.2)
            self.arm4.setPosition(-pi/2)
            self.arm5.setPosition(pi/2)
        else:
            print("arm_height() called with a wrong argument\n")

        self.current_height = ht

    def arm_set_orientation(self, orien):
        if orien == self.orientation['ARM_BACK_LEFT']:
            self.arm1.setPosition(-2.949)
        elif orien == self.orientation['ARM_LEFT']:
            self.arm1.setPosition(-pi/2)
        elif orien == self.orientation['ARM_FRONT_LEFT']:
            self.arm1.setPosition(-0.2)
        elif orien == self.orientation['ARM_FRONT']:
            self.arm1.setPosition(0.0)
        elif orien == self.orientation['ARM_FRONT_RIGHT']:
            self.arm1.setPosition(0.2)
        elif orien == self.orientation['ARM_RIGHT']:
            self.arm1.setPosition(pi/2)
        elif orien == self.orientation['ARM_BACK_RIGHT']:
            self.arm1.setPosition(2.949)
        else:
            print("arm_set_side() called with a wrong argument")

        self.current_orientation = orien
    
    def arm_increase_height(self):
        self.current_height += 1
        if self.current_height >= self.height['ARM_MAX_HEIGHT']:
            self.current_height = (self.height['ARM_MAX_HEIGHT'] - 1)
        self.arm_set_height(self.current_height)

    def arm_decrease_height(self):
        self.current_height -= 1
        if int(self.current_height) < 0:
            self.current_height = 0
        self.arm_set_height(self.current_height)

    def arm_increase_orientation(self):
        self.current_orientation += 1
        if self.current_orientation >= self.orientation['ARM_MAX_SIDE']:
            self.current_orientation = (self.orientation['ARM_MAX_SIDE'] - 1)
        self.arm_set_orientation(self.current_orientation)

    def arm_decrease_orientation(self):
        self.current_orientation -= 1
        if int(self.current_orientation < 0):
            self.current_orientation = 0
        self.arm_set_orientation(self.current_orientation)

    def arm_set_sub_arm_rotation(self, arm, radian):
        arm.setPosition(radian)

    def arm_get_sub_arm_length(self, arm):
        if arm == 'arm1':
            return 0.253
        elif arm == 'arm2':
            return 0.155
        elif arm == 'arm3':
            return 0.135
        elif arm == 'arm4':
            return 0.081
        elif arm == 'arm5':
            return 0.105
        else:
            return 0.0

    def arm_ik(self, x, y, z):
        x1 = sqrt(x * x + z * z)
        y1 = y + self.arm_get_sub_arm_length('arm4') + self.arm_get_sub_arm_length('arm5') - self.arm_get_sub_arm_length('arm1')

        a = self.arm_get_sub_arm_length('arm2')
        b = self.arm_get_sub_arm_length('arm3')
        c = sqrt(x1 * x1 + y1 * y1)

        alpha = -asin(z/x1)
        beta = -(pi/2 - acos((a * a + c * c - b * b) / (2.0 * a * c)) - atan(y1 / x1))
        gamma = -(pi - acos((a * a + b * b - c * c) / (2.0 * a * b)))
        delta = -(pi + (beta + gamma))
        epsilon = pi/2 + alpha;

        self.arm1.setPosition(alpha)
        self.arm2.setPosition(beta)
        self.arm3.setPosition(gamma)
        self.arm4.setPosition(delta)
        self.arm5.setPosition(epsilon)



    

    
