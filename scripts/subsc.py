#!/usr/bin/env python3

import rospy
import moveit_commander
from geometry_msgs.msg import Point,Pose
from tf.transformations import quaternion_from_euler


def main():

    rospy.init_node("subscribe")                       # node to start
    crane = moveit_commander.RobotCommander()          # crane instantiation
    print("")
    print("Robot Group names:  ", end="")
    print(crane.get_group_names())                     # crane view [ all Group name ]
    print("Robot current state:")
    print(crane.get_current_state())                   # crane view current_state
    arm = moveit_commander.MoveGroupCommander("arm")   # arm instantiation
    arm.set_max_velocity_scaling_factor(0.1)           # arm setting_max_velocity
    arm.set_max_acceleration_scaling_factor(1.0)       # arm setting_max_acceleration
    gripper = moveit_commander.MoveGroupCommander("gripper")    # gripper instantiation

    arm_init_pose = arm.get_current_pose().pose        # arm view initial_pose
    print("")
    print("Arm initial pose:")
    print(arm_init_pose)
    print("home")
    arm.set_named_target("home")
    arm.go()                                           # arm moving to "home"

    get_rpy = arm.get_current_rpy()                    # arm getting to [ roll, pitch, yaw ]
    print(get_rpy)
    get_xyz = arm.get_current_pose().pose              # arm getting to pose
    print(get_xyz)
    print("okokokokokok")

    new_rpy = get_rpy
    new_arr = [
            get_xyz.position.x + 0,
            get_xyz.position.y + 0.0,
            get_xyz.position.z + 0.0,
            get_rpy[0],
            get_rpy[1],
            get_rpy[2] ]

    print(new_arr)
    arm.set_pose_target( new_arr )
    arm.go()
    print("new_set")
    get_test = arm.get_current_rpy() 
    print(get_test)
    get_xyz = arm.get_current_pose()
    print(get_xyz)
    print("uiuiuuuuiui")

    rospy.Subscriber("mediapipe_difference", Point, callback)

    #rospy.sleep(0.1)
    rospy.spin()


def callback(msg):

    X_GAIN = 0.0001
    Y_GAIN = 0.0001
    Z_GAIN = 0.0001

    print("publish  :  ", end="")
    print(msg.x, msg.y, msg.z)
    sub_diff = Point()
    sub_diff = msg
    print("sub_diff :  ", end="")
    print(sub_diff.x, sub_diff.y, sub_diff.z)

    fix_x = sub_diff.z * Z_GAIN
    fix_y = sub_diff.x * X_GAIN
    fix_z = sub_diff.y * Y_GAIN

    arm = moveit_commander.MoveGroupCommander("arm")
    arm_current_pose = arm.get_current_pose().pose
    print("arm_current_pose  :  ", end="")
    print(arm_current_pose.position.x, " , ",  arm_current_pose.position.y, " , " , arm_current_pose.position.z)
    #arm_current_pose.position.x = arm_current_pose.position.x + fix_x
    #arm_current_pose.position.y = arm_current_pose.position.y + fix_y
    #arm_current_pose.position.z = arm_current_pose.position.z + fix_z
    #arm.set_pose_target( arm_current_pose ) # 目標ポーズ設定
    #arm.go()


    target_pose = Pose()
    target_pose.position.x = arm_current_pose.position.x + 0.00
    target_pose.position.y = arm_current_pose.position.y + 0.00
    target_pose.position.z = arm_current_pose.position.z + 0.00
    print("target_pose       :  ", end="")
    print(target_pose.position.x, " , ", target_pose.position.y, " , " ,target_pose.position.z)
    #target_pose.position.x = 0.0
    #target_pose.position.y = 0.0
    #target_pose.position.z = test

    #q = quaternion_from_euler(0.0,0.0,0.0)

    target_pose.orientation.x = arm_current_pose.orientation.x
    target_pose.orientation.y = arm_current_pose.orientation.y
    target_pose.orientation.z = arm_current_pose.orientation.z
    target_pose.orientation.w = arm_current_pose.orientation.w
    
    #print(arm_current_pose.position.z)

    #target_pose.orientation.x = q[0]
    #target_pose.orientation.y = q[1]
    #target_pose.orientation.z = q[2]
    #target_pose.orientation.w = q[3]
    arm_current_rpy = arm.get_current_rpy() 
    print("arm_current_rpy  :  ", end="")
    print(arm_current_rpy)

    target = [
            arm_current_pose.position.x + 0.00,
            arm_current_pose.position.y + 0.00, 
           arm_current_pose.position.z + 0.001,
            arm_current_rpy[0],
            arm_current_rpy[1],
            arm_current_rpy[2] ]
    print("target  :  ", end="")
    print(target)

    #rospy.loginfo(target)
    arm.set_pose_target( target )
    arm.go()

    #test += 0.01

    print("============================================================")


if __name__ == "__main__":
    main()
