#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import actionlib
import math
import tf
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

    rospy.init_node("arn")
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.4)
    arm.set_max_acceleration_scaling_factor(1.0)
    gripper = actionlib.SimpleActionClient("crane_x7/gripper_controller/gripper_cmd", GripperCommandAction)
    gripper.wait_for_server()
    gripper_goal = GripperCommandGoal()
    gripper_goal.command.max_effort = 5.0 # torque
    print("=== finish init ===")

    print(" setting home ")
    arm.set_named_target("home")
    arm.go()
    rospy.sleep(1.0)
    print("=== finish ===")

    print(" open gripper ")
    gripper_goal.command.position = OPEN_RADIAN
    gripper.send_goal(gripper_goal)
    gripper.wait_for_result(rospy.Duration(1.0))
    print("=== finish ===")
    
    print(" gripping fork, while 10 seconds ... ")
    rospy.sleep(3.0)
    gripper_goal.command.position = GRIP_RADIAN
    gripper.send_goal(gripper_goal)
    gripper.wait_for_result(rospy.Duration(1.0))
    rospy.sleep(3.0)
    print("=== finish ===")

    print(" setting SEARCH_POSE ")
    arm.set_pose_target(SEARCH_POSE)
    arm.go()
    rospy.sleep(3.0)
    print("=== finish ===")


    rospy.Subscriber("/tf", TFMessage, selection, queue_size=1)
    

    rospy.spin()

def selection(msg):
    buff = msg
    print("msg========")
    print(msg)
    print("buff++++++++")
    print(buff)
    





if __name__ == '__main__':
    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass

