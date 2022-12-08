#!/usr/bin/env python3

import rospy
import moveit_commander


def main():
    rospy.init_node("pra")                       # ROS node を実行
    crane = moveit_commander.RobotCommander()    # RobotCommander をインスタンス化

    print("")
    print("Robot Group names:  ", end="")        # crane_x7 の Group 全ての名前
    print(crane.get_group_names())

    print("Robot current state:")      # crane_x7 の現在の状態
    print(crane.get_current_state())

    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.1)
    arm.set_max_acceleration_scaling_factor(1.0)

    gripper = moveit_commander.MoveGroupCommander("gripper")

    arm_init_pose = arm.get_current_pose().pose  # arm の初期姿勢
    print("")
    print("Arm initial pose:  ", end="")
    print(arm_init_pose)

    print("home")
    arm.set_named_target("home")
    arm.go()

    print("vertical")
    arm.set_named_target("vertical")
    arm.go()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

