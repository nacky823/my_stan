#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import actionlib
import tf
from tf import transformations
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point, Pose
from control_msgs.msg import GripperCommandAction, GripperCommandGoal

mediapipe_coordinate = Pose()
gazebo_model_states = ModelStates()

def callback(msg):
    global mediapipe_coordinate
    mediapipe_coordinate.position.x = msg.x
    mediapipe_coordinate.position.y = msg.y
    mediapipe_coordinate.position.z = msg.z
    #print(msg)
    #print(mediapipe_coordinate)

def cb(msg):
    global gazebo_model_states
    gazebo_model_states = msg
    print(gazebo_model_states)

def soiya(msg):
    global tf_q
    tf_q = msg
    print(tf_q)

def main():
    global mediapipe_coordinate
    OBJECT_NAME = "wood_cube_5cm"

    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.4)
    arm.set_max_acceleration_scaling_factor(1.0)
    gripper = actionlib.SimpleActionClient("crane_x7/gripper_controller/gripper_cmd", GripperCommandAction)
    gripper.wait_for_server()
    print(gripper)
    gripper_goal = GripperCommandGoal()
    gripper_goal.command.max_effort = 4.0
    print(gripper_goal)

    arm.set_named_target("home")
    arm.go()
    rospy.sleep(1.0)

    sleep_time = 1.0
    print("Wait " + str(sleep_time) + " secs.")
    rospy.sleep(sleep_time)
    print("Start")

    tf_listener = tf.TransformListener()
    print(tf_listener)

    #sub_model_states = rospy.Subscriber("gazebo/model_states", ModelStates, cb, queue_size=1)

    #tf_a = tf_listener.lookupTransform('/base_link',name,rospy.Time(0))
    print(gazebo_model_states)
    rospy.Subscriber("/tf", Pose, soiya, queue_size=1)

    if OBJECT_NAME in gazebo_model_states.name:
            object_index = gazebo_model_states.name.index(OBJECT_NAME)
            print(hoge)
            print(object_index)




    #rospy.Subscriber("mediapipe/mouth_coordinate", Point, callback, queue_size=10)

    #rospy.spin()

if __name__ == '__main__':
    rospy.init_node("new")

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
