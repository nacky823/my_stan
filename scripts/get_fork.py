#!/usr/bin/env python3
# coding: utf-8
# get_fork.py

# SPDX-FileCopyrightText: 2022 YAZAWA Kenichi s21c1036hn@s.chibakoudai.jp
# SPDX-License-Identifier: 

# グリッパーを開く
# キーが入力されるまで開いたまま
# キーが入力されたらグリッパーを閉じる

import rospy
import math
import sys
import moveit_commander

def deg2rad(deg):
    return math.radians(deg)

def main():
    # 閉まる角度を指定 1 未満にするとなぜか範囲外だと言われる
    deg = 1
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
    # コードを終了する
    return 0

if __name__ == "__main__":
    res = main()
    sys.exit(res)

