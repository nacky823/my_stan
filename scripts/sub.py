#!/usr/bin/env python3

import rospy
import moveit_commander
from geometry_msgs.msg import Point
from tf.transformations import quaternion_from_euler

def main():

    rospy.init_node("subscribe")
    crane = moveit_commander.RobotCommander()    # RobotCommander をインスタンス化
    print("Robot current state:")                # crane_x7 の現在の状態
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.1)
    arm.set_max_acceleration_scaling_factor(1.0)
    gripper = moveit_commander.MoveGroupCommander("gripper")
    arm_init_pose = arm.get_current_pose().pose  # arm の初期姿勢
    print("")
    print("Arm initial pose:")
    print(arm_init_pose)

    rospy.spin()

if __name__ == '__main__':
    main()
















