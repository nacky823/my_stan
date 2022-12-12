#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point

def main():

    rospy.init_node("subscribe")
    rospy.Subscriber("mediapipe_difference", Point, callback)

    #rospy.sleep(0.1)
    rospy.spin()

def callback(msg):

    X_GAIN = 0.1
    Y_GAIN = 0.1
    Z_GAIN = 0.1

    rospy.loginfo(msg)
    sub_diff = Point()
    sub_diff = msg
    rospy.loginfo(sub_diff)
    rospy.loginfo("===================================================================")

    fix_x = sub_diff.x * X_GAIN
    rospy.loginfo("recieved %f", float(fix_x))
    fix_y = sub_diff.y * Y_GAIN
    rospy.loginfo("recieved %f", float(fix_y))
    fix_z = sub_diff.z * Z_GAIN
    rospy.loginfo("recieved %f", float(fix_z))


















if __name__ == "__main__":
    main()
