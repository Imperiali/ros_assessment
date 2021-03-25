#!/usr/bin/env python3

import rospy
import sys
from std_msgs.msg import Int32
from std_msgs.msg import Float64
import numpy as np


def move_arm(height):
    arm = rospy.Publisher(
        'rrbot/joint_base_mid_position_controller/command', Float64, queue_size=10)

    movement = Float64()
    movement.data = height

    arm.publish(movement)


def callback(data):
    ARM_DOWN = -0.1
    ARM_UP = 0.02

    print(data.data)

    move_arm(ARM_DOWN if data.data == 1 else ARM_UP)


def control():
    rospy.init_node('bot_control', anonymous=True)

    # rate = rospy.Rate(30)

    while not rospy.is_shutdown():

        # move_arm(ARM_DOWN if direction == 1 else ARM_UP)
        rospy.Subscriber('bot_arm_control', Int32, callback)

        rospy.spin()


# direction = int(sys.argv[1])
# direction = rospy.get_param('direction')
# print('direction')
# print(direction)

if __name__ == '__main__':
    print('Controler called')
    control()
    '''
    try:
        control_robot()
    except:
        print('Node ended.')
    '''
