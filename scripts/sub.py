#!/usr/bin/env python3

import rospy
import moveit_commander
from geometry_msgs.msg import Point
from tf.transformations import quaternion_from_euler



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

    get_rpy = arm.get_current_rpy() 
    print(get_rpy)
    get_xyz = arm.get_current_pose().pose
    print(get_xyz)
    print("okokokokokok")

    new_rpy = get_rpy
    new_arr = [
            get_xyz.position.x + 0.1,
            get_xyz.position.y + 0.1,
            get_xyz.position.z,
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

    X_GAIN = 0.001
    Y_GAIN = 0.001
    Z_GAIN = 0.001

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

    arm = moveit_commander.MoveGroupCommander("arm")
    arm_current_pose = arm.get_current_pose().pose  # arm の現在姿勢$
    arm_current_pose.position.x = arm_current_pose.position.x + fix_x
    arm_current_pose.position.y = arm_current_pose.position.y + fix_y
    arm_current_pose.position.z = arm_current_pose.position.z + fix_z
    #arm.set_pose_target( arm_current_pose ) # 目標ポーズ設定
    #arm.go()

    arm_current_rpy = arm.get_current_rpy() 
    
    target = [
            arm_current_pose.position.x,
            arm_current_pose.position.y,
            arm_current_pose.position.z,
            arm_current_rpy[0],
            arm_current_rpy[1],
            arm_current_rpy[2] ]

    rospy.loginfo(target)
    arm.set_pose_target( target )
    arm.go()




















if __name__ == "__main__":
    main()