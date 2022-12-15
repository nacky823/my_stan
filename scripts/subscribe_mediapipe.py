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
    arm_current_pose = arm.get_current_pose().pose  # arm の現在姿勢$
    print("")
    print("Arm current pose:")
    print(arm_current_pose)
    print("")
    current_joint_deg = gripper.get_current_joint_values()
    print("Joint current deg:")
    print(current_joint_deg)
    current_arm_deg = arm.get_current_joint_values()
    print("Arm current deg:")
    print(current_arm_deg)
    set_arm_deg = [
            0.0019240830344822157,
            0.3816045993673516,
            0.0009249622074509745,
            -2.2298474391216097,
            0.05583576835324777,
            0.2814626521875107,
            -0.00464467406541047]
    arm.set_joint_value_target(set_arm_deg)
    arm.go()
    arm_current_pose = arm.get_current_pose().pose  # arm の現在姿勢$
    print("")
    print("Arm current pose:")
    print(arm_current_pose)
    print(arm_current_pose.position.x)
    arm_current_pose = arm.get_current_pose().pose  # arm の現在姿勢$
    arm_current_pose.position.x = arm_current_pose.position.x + 2
    arm_current_pose.position.y = arm_current_pose.position.y + 2
    arm_current_pose.position.z = arm_current_pose.position.z + 0
    print(arm_current_pose)
    arm_change_pose = [
            arm_current_pose.position.x,
            arm_current_pose.position.y,
            arm_current_pose.position.z,
            arm_current_pose.orientation.x,
            arm_current_pose.orientation.y,
            arm_current_pose.orientation.z,
            arm_current_pose.orientation.w ]
    print(arm_change_pose)
    arm_current_pose.position.x = 0.0
    arm_current_pose.position.y = 0.0
    arm_current_pose.position.z = 0.8
    q = quaternion_from_euler( 0.0, 0.0, 0.0 )
    arm_current_pose.orientation.x = q[0]
    arm_current_pose.orientation.y = q[1]
    arm_current_pose.orientation.z = q[2]
    arm_current_pose.orientation.w = q[3]

    test = [
            0,
            0,
            0.3,
            1.6,
            -0.03,
            -1.6 ]


    arm.set_pose_target( test ) # 目標ポーズ設定
    arm.go()
    #get_test = arm.shift_pose_target() 
    get_test = arm.get_current_rpy() 
    print(get_test)
    get_xyz = arm.get_current_pose()
    print(get_xyz)
    print("okokokokokok")


    rospy.Subscriber("mediapipe_difference", Point, callback)

    #rospy.sleep(0.1)
    rospy.spin()

def callback(msg):

    X_GAIN = 1
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

    arm = moveit_commander.MoveGroupCommander("arm")
    arm_current_pose = arm.get_current_pose().pose  # arm の現在姿勢$
    arm_current_pose.position.x = arm_current_pose.position.x + fix_x
    arm_current_pose.position.y = arm_current_pose.position.y + fix_y
    arm_current_pose.position.z = arm_current_pose.position.z + fix_z
    arm.set_pose_target( arm_current_pose ) # 目標ポーズ設定
    arm.go()


















if __name__ == "__main__":
    main()
