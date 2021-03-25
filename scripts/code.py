#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseActionResult
from tf.transformations import quaternion_from_euler
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


if __name__ == '__main__':

    status = 0
    pose = set_goal()
    rospy.init_node('pose_goal')

    talker_main(pose)
    sleep(2)
    while not rospy.is_shutdown():
        rospy.Subscriber('/move_base/result',
                         MoveBaseActionResult, listener_callback)
        if status == 3:
            break
