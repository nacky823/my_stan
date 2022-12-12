#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point

def main():

    X_GAIN = 0.1

    rospy.init_node("subscribe")
    rospy.Subscriber("mediapipe_difference", Point, callback)

    while not rospy.is_shutdown():

        #fix_x = sub_diff.x * X_GAIN
        #rospy.loginfo("recieved %f", float(fix_x))



        rospy.sleep(0.1)

def callback(msg):
    rospy.loginfo(msg)
    sub_diff = Point()
    sub_diff = msg
    rospy.loginfo("recieved")
    rospy.loginfo(sub_diff)



















if __name__ == "__main__":
    main()
