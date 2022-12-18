#!/usr/bin/env python3
# coding: utf-8
# thrust.py

# SPDX-FileCopyrightText: 2022 nacky823 | YAZAWA Kenichi s21c1036hn@s.chibakoudai.jp
# SPDX-License-Identifier: MIT License

import rospy
import moveit_commander
from geometry_msgs.msg import Point,Pose
from tf.transformations import quaternion_from_euler
import math
import numpy as np
import sys
from thrust import arm_thrust_node 

def deg2rad(deg):
    return math.radians(deg)

# get_fork をそのまま持ってきた
def get_fork():
    # 閉まる角度を指定 1 未満にするとなぜか範囲外だと言われる
    deg = 0.8
    GRIPPER_CLOSE = [deg2rad(deg), deg2rad(deg)]
    # 開く角度を指定 度数法 93 度が限界
    deg = 15
    GRIPPER_OPEN = [deg2rad(deg), deg2rad(deg)]
    # ノードを初期化する
    rospy.init_node('get_fork')
    # gripper グループを取得する
    gripper = moveit_commander.MoveGroupCommander("gripper")
    # 最大関節速度を下げるらしい 0 ~ 1 で指定
    gripper.set_max_velocity_scaling_factor(0.4)
    # 最大関節加速度を下げるらしい 0 ~ 1 で指定
    gripper.set_max_acceleration_scaling_factor(1.0)
    # 目標値を設定
    gripper.set_joint_value_target(GRIPPER_OPEN)
    # 目標値に向けて動かす
    gripper.go()
    # キーが入力されるまで待つ
    input("Input Key for Gripper Close ... ")
    # キーが入力されたらグリッパーを閉じる
    gripper.set_joint_value_target(GRIPPER_CLOSE)
    # 目標値に向けて動かす
    gripper.go()

class go_mouth_node():
    def __init__(self):
        # atn => Arm Thrust Node
        self.generate_subscriber()
        self.atn = arm_thrust_node("beside")
        # self.atn = arm_thrust_node("down")
        self.count = 0

    # サブスクライバを作成する
    def generate_subscriber(self):
        print("topic name : /mediapipe_difference")
        self.sub = rospy.Subscriber("/mediapipe_difference", Pose, self.callback)

    # 一回目だけ実行する
    def callback(self, target_pose):
        if not self.count > 0:
            # エイムしたあとに下に突き出す
            print("callbacked")
            self.atn.main(self.pose2position(target_pose))
        self.count += 1

    # 何回コールバックしたか取得する
    def get_count(self):
        return self.count

    # コールバックの回数をリセットする
    def reset_count(self):
        self.count = 0

    # ポーズからポジションを取り出す
    def pose2position(self, _pose):
        return [_pose.position.x, _pose.position.y, _pose.position.z]

    # パブリッシャが無い時にテスト実行する時に呼び出すもの
    def tester(self, _coord):
        target_pose = Pose()

        target_pose.position.x = _coord[0]
        target_pose.position.y = _coord[1]
        target_pose.position.z = _coord[2]
        target_pose.orientation.x = 0
        target_pose.orientation.y = 0
        target_pose.orientation.z = 0
        target_pose.orientation.w = 1

        self.callback(target_pose)

if __name__ == "__main__":
    rospy.init_node("go_mouth_node")
    get_fork()
    gmn = go_mouth_node()

    while not gmn.get_count() > 0:
        # gmn.tester([-0.2, 0.2, 0.3])
        rospy.sleep(0.1)

    print(gmn.get_count())

    del gmn

    # 終了コード
    sys.exit(0)

