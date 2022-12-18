#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from tf import transformations
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point, Pose
from tf2_msgs.msg import TFMessage


def main():

    rospy.Subscriber("/tf", TFMessage, food)

    rospy.spin()

def food(msg):
    food = Pose()
    i = 0
    for lis in msg.transforms:
        if lis.child_frame_id == "ar_marker_0" :
            food.position.x = msg.transforms[i].transform.translation.x
            food.position.y = msg.transforms[i].transform.translation.y
            food.position.z = msg.transforms[i].transform.translation.z
            food.orientation.x = msg.transforms[i].transform.rotation.x
            food.orientation.y = msg.transforms[i].transform.rotation.y
            food.orientation.z = msg.transforms[i].transform.rotation.z
            food.orientation.w = msg.transforms[i].transform.rotation.w
            print(food.position.x)
            print("x")
            print(food.position.y)
            print("y")
            print(food.position.z)
            print("z")
            print(food.orientation.x)
            print("qx")
            print(food.orientation.y)
            print("qy")
            print(food.orientation.z)
            print("qz")
            print(food.orientation.w)
            print("qw")
            #print(msg.transforms[i].transform.translation.x)
        i += 1
    food = rospy.Publisher("/food_coordinaite", Point, queue_size=10)

        


#def selection(msg):
#    i = 0
    #for child_frame_id in msg.transforms:
#        if (child_frame_id.child_frame_id == "ar_marker_0"):
            
            #food.position.x = msg.transforms[i].transform.translation.x
            #food.position.y = msg.transforms[i].transform.translation.y

    #        print(msg.transforms[i].transform.translation.x)
    #    indent +=1




if __name__ == '__main__':
    rospy.init_node('ar2')
    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass

