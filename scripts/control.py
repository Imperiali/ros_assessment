#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseActionResult
import numpy as np
from time import sleep


def set_goal():

    pose_x = eval(
        input("Digite a posição desejada no eixo X com formato 0.0: "))
    pose_y = eval(
        input("Digite a posição desejada no eixo Y com formato 0.0: "))
    pose_theta = eval(
        input("Digite a posição desejada no eixo Theta com formato 0.0: "))

    goals = PoseStamped()
    goals.header.frame_id = 'map'
    goals.pose.position.x = pose_x
    goals.pose.position.y = pose_y
    quat = quaternion_from_euler(0, 0, pose_theta)
    goals.pose.orientation.x = quat[0]
    goals.pose.orientation.y = quat[1]
    goals.pose.orientation.z = quat[2]
    goals.pose.orientation.w = quat[3]

    return goals


def listener_callback(data):
    global status
    status = data.status.status


def talker_main(goal):

    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rate = rospy.Rate(4)

    if not rospy.is_shutdown():
        rospy.loginfo(goal)
        pub.publish(goal)
        rate.sleep()
        # O destino só é aceito após a segunda publicação (?)
        rospy.loginfo(goal)
        pub.publish(goal)


def move_bot():
    global status
    pose = set_goal()

    WAFFLE_MAX_LIN_VEL = 0.26
    WAFFLE_MAX_ANG_VEL = 1.82

    talker_main(pose)
    sleep(2)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    twist = Twist()
    twist.linear.x = WAFFLE_MAX_LIN_VEL
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = WAFFLE_MAX_ANG_VEL

    while not rospy.is_shutdown():
        rospy.Subscriber('/move_base/result',
                         MoveBaseActionResult, listener_callback)
        pub.publish(twist)
        print(status)
        if status == 3:
            break


def move_arm(height):
    arm = rospy.Publisher('/bot_arm_control', Int32, queue_size=10)

    direction = Int32()
    direction.data = height

    arm.publish(direction)


def set_initial_pose():
    pub = rospy.Publisher(
        '/initialpose', PoseWithCovarianceStamped, queue_size=10)

    rate = rospy.Rate(20)

    horizontal_position = -2.0
    vertical_position = -0.5
    angle = 0.0

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.pose.pose.position.x = horizontal_position
    msg.pose.pose.position.y = vertical_position
    quat = quaternion_from_euler(0, 0, angle)
    msg.pose.pose.orientation.x = quat[0]
    msg.pose.pose.orientation.y = quat[1]
    msg.pose.pose.orientation.z = quat[2]
    msg.pose.pose.orientation.w = quat[3]

    pub.publish(msg)
    rate.sleep()
    pub.publish(msg)


def control():
    ARM_DOWN = 1
    ARM_UP = 2

    rospy.init_node('bot_controler', anonymous=True)

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():

        move_arm(ARM_UP)

        rate.sleep()

        move_arm(ARM_DOWN)


if __name__ == '__main__':
    status = 0
    print('Controler called')
    control()
    '''
    try:
        control_robot()
    except:
        print('Node ended.')
    '''
