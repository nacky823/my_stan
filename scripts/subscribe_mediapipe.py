#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point

def callback(msg):
    rospy.loginfo(msg)

def main():

    rospy.init_node("subscribe")
    rospy.Subscriber("mediapipe_difference", Point, callback)























    rospy.spin()    # ノードが終了するまで待機


if __name__ == "__main__":
    main()
