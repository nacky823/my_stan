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

class get_food_node():
    def __init__(self):
        # atn => Arm Thrust Node
        self.generate_subscriber()
        self.atn = arm_thrust_node("down")
        # self.atn = arm_thrust_node("beside")
        self.count = 0

    # サブスクライバを作成する
    def generate_subscriber(self):
        print("topic name : /food_coordinate")
        self.sub = rospy.Subscriber("/food_coordinate", Pose, self.callback)

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
    rospy.init_node("get_food_node")
    gfn = get_food_node()

    while not gfn.get_count() > 0:
        # gfn.tester([0.2, 0.2, 0.085])
        rospy.sleep(0.1)

    print(gfn.get_count())

    del gfn

    # 終了コード
    sys.exit(0)

