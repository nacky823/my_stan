#!/usr/bin/env python3

import rospy
import moveit_commander
from geometry_msgs.msg import Point,Pose
from tf.transformations import quaternion_from_euler
import math

def main():
    rospy.init_node("subscribe")                       # node to start
    crane = moveit_commander.RobotCommander()          # crane instantiation
    print("")
    print("Robot Group names:  ", end="")
    print(crane.get_group_names())                     # crane view [ all Group name ]
    print("Robot current state:")
    print(crane.get_current_state())                   # crane view current_state
    arm = moveit_commander.MoveGroupCommander("arm")   # arm instantiation
    arm.set_max_velocity_scaling_factor(0.7)           # arm setting_max_velocity
    arm.set_max_acceleration_scaling_factor(1.0)       # arm setting_max_acceleration
    gripper = moveit_commander.MoveGroupCommander("gripper")    # gripper instantiation

    # "vertical" に一旦戻す
    arm.set_named_target("vertical")
    arm.go()

    arm_init_pose = arm.get_current_pose().pose        # arm view initial_pose
    """ Default
    home_pose = [
            0.165929,
            0.000055,
            0.272437,
            0.236961,
            1.361300,
            0.218571
            ]
    """
    home_pose = [
            - 0.165929,
            0.000055,
            0.272437,
            math.pi / 2,
            0,
            - math.pi / 2
            ]
    print("")
    print("Arm initial pose:")
    print(arm_init_pose)
    print("search_pose_values")
    arm.set_pose_target(home_pose)
    arm.go()                                           # arm moving to "home"
    """
    _current_joint_values = arm.get_current_joint_values()
    _current_joint_values[-1] = math.pi / 2
    arm.set_joint_value_target(_current_joint_values)
    arm.go()
    """

    get_rpy = arm.get_current_rpy()                    # arm getting to [ roll, pitch, yaw ]
    print(get_rpy)
    get_xyz = arm.get_current_pose().pose              # arm getting to pose
    print(get_xyz)
    print("okokokokokok")

    new_rpy = get_rpy
    new_arr = [
            get_xyz.position.x + 0.0,
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

    arm = moveit_commander.MoveGroupCommander("arm")            # arm instantiation
    gripper = moveit_commander.MoveGroupCommander("gripper")    # gripper instantiation
    # 現在の位置姿勢を取得
    arm_current_pose = arm.get_current_pose()
    print("arm_current_pose  :  ", end="")
    print(arm_current_pose.pose.position.x, " , ",  arm_current_pose.pose.position.y, " , " , arm_current_pose.pose.position.z)
    #print(arm_current_pose)

    # 次の位置姿勢を計算
    arm_target_pose = Pose()
    arm_target_pose.position.x = arm_current_pose.pose.position.x - fix_x
    arm_target_pose.position.y = arm_current_pose.pose.position.y - fix_y
    arm_target_pose.position.z = arm_current_pose.pose.position.z - fix_z #0.05
    print("arm_target_pose  :  ", end="")
    print(arm_target_pose.position.x, " , ",  arm_target_pose.position.y, " , " , arm_target_pose.position.z)

    # 現在のエンドエフェクタの姿勢をロールピッチヨー角で取得する
    arm_current_rpy = arm.get_current_rpy() 
    print("arm_current_rpy  :  ", end="")
    print(arm_current_rpy)

    # この値に go する
    target = [
            # 位置は更新
            arm_target_pose.position.x,
            arm_target_pose.position.y,
            arm_target_pose.position.z,
            # 姿勢はそのまま
            arm_current_rpy[0],
            arm_current_rpy[1],
            arm_current_rpy[2] ]
    print("target  :  ", end="")
    print(target)

    arm.set_pose_target( target )
    arm.go()
    print("arm_current_pose  :  ", end="")
    print(arm_current_pose.pose.position.x, " , ",  arm_current_pose.pose.position.y, " , " , arm_current_pose.pose.position.z)
    print("arm_current_rpy  :  ", end="")
    print(arm_current_rpy)

    print("============================================================")


if __name__ == "__main__":
    main()
