#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
from geometry_msgs.msg import Point, Pose

mediapipe_coordinate = Pose()

def callback(msg):
    global mediapipe_coordinate
    print(msg)
    mediapipe_coordinate.position.x = msg.x
    mediapipe_coordinate.position.y = msg.y
    mediapipe_coordinate.position.z = msg.z
    print(mediapipe_coordinate)

def main():
    global mediapipe_coordinate

    rospy.Subscriber("mediapipe/mouth_coordinate", Point, callback, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    rospy.init_node("new")

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
