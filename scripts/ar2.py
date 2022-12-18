#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import actionlib
import math
from tf import transformations
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point, Pose
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from tf2_msgs.msg import TFMessage

food = Pose()
mouth = Pose()

def callback_food(msg):
    global food
    food = msg
    print("msg:")
    print(msg)
    print("food:")
    print(food)

def callback_mouth(msg):
    global mouth
    mouth = msg
    print("msg:")
    print(msg)
    print("mouth:")
    print(mouth)

def arn(target):
    print(target)


def main():
    global food
    global mouth

    OPEN_RADIAN = 0.2
    GRIP_RADIAN = 0.1
    SEARCH_POSE = [ 0.23907234769857613, -0.002097075232802146, 0.31566854613745765,
            -0.6305574889386819, 0.6358549451438162, -0.31570555911429393, 0.313712833374456 ]


    #get_cnt = 0
    #while not get_cnt >= 5 :
    rospy.Subscriber("/tf", TFMessage, selection)
    #rospy.Subscriber("/ar_pose_marker", markers, selection)
        #print(sub)
        
        #get_cnt = selection()
        #rospy.Subscriber("/ar_pose_marker", TFMessage, selection)
    

    rospy.spin()

def selection(msg):
    food = msg
    #print("msg========")
    #print(msg)
    #print("buff++++++++")
    #print(food)
    #print("frame_id~~~~~~")
    #print(buff.transforms.child_frame_id)
    #print("buff.position.x====")
    #print(buff.position.x)
    indent = 0
    for child_frame_id in msg.transforms:
        if (child_frame_id.child_frame_id == "ar_marker_0"):
            print(msg.transforms[indent].transform.translation.x)
        indent +=1




if __name__ == '__main__':
    rospy.init_node('ar2')
    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass

