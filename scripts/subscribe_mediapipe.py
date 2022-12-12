#!/usr/bin/env python3

import rospy
import moveit_commander
from geometry_msgs.msg import Point

def main():

    rospy.init_node("subscribe")
    crane = moveit_commander.RobotCommander()    # RobotCommander をインスタンス化
    print("")
    print("Robot Group names:  ", end="")        # crane_x7 の Group 全ての名前
    print(crane.get_group_names())
    print("Robot current state:")                # crane_x7 の現在の状態
    print(crane.get_current_state())
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.1)
    arm.set_max_acceleration_scaling_factor(1.0)
    gripper = moveit_commander.MoveGroupCommander("gripper")
    arm_init_pose = arm.get_current_pose().pose  # arm の初期姿勢
    print("")
    print("Arm initial pose:")
    print(arm_init_pose)
    print("home")
    arm.set_named_target("home")
    arm.go()
    arm_current_pose = arm.get_current_pose().pose  # arm の現在姿勢$
    print("")
    print("Arm current pose:")
    print(arm_current_pose)

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
