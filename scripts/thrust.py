#!/usr/bin/env python3
# coding: utf-8
# thrust.py

# SPDX-FileCopyrightText: 2022 nacky823 | YAZAWA Kenichi s21c1036hn@s.chibakoudai.jp
# SPDX-License-Identifier: MIT License

import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
import math
import numpy as np
import sys

# 二点間の距離を求める
def norm(a, b):
    return np.linalg.norm(np.subtract(a, b), ord = 2)

# リストの要素同士の差を計算する
def diff(a, b):
    new = []
    for i, v in enumerate(a):
        new.append(a[i] - b[i])
    return new

# エンドエフェクタを平行移動するためだけのクラス
class liner_movement():
    # インスタンス化する時にエンドエフェクタのグループを渡す必要がある
    def __init__(self, end_effector_group):
        self.end_effector = end_effector_group

    # エンドエフェクタの移動量を指定する関数
    def move_effector_translate(self, x, y, z):
        # 現在の位置姿勢を取得
        end_effector_current_pose = self.end_effector.get_current_pose()

        # 次の位置姿勢を計算
        end_effector_target_pose = Pose()

        # 位置を加算
        end_effector_target_pose.position.x = end_effector_current_pose.pose.position.x + x
        end_effector_target_pose.position.y = end_effector_current_pose.pose.position.y + y
        end_effector_target_pose.position.z = end_effector_current_pose.pose.position.z + z

        # 現在のエンドエフェクタの姿勢をロールピッチヨー角で取得する
        end_effector_current_rpy = self.end_effector.get_current_rpy() 

        # この値に go する
        target = [
                # 位置は更新
                end_effector_target_pose.position.x,
                end_effector_target_pose.position.y,
                end_effector_target_pose.position.z,
                # 姿勢はそのまま
                end_effector_current_rpy[0],
                end_effector_current_rpy[1],
                end_effector_current_rpy[2] ]

        self.end_effector.set_pose_target( target )
        self.end_effector.go()

    # エンドエフェクタを指定量だけ前または後ろに動かすだけの動作
    def z_translate(self, z):
        # 一方向のみに移動。行ったら行きっぱなし
        self.move_effector_translate(0, 0, z)

    # 指定量だけ前に動かして、指定量だけ後ろに動かすだけの動作
    def z_updn(self, z):
        # 行ったら同じところに返ってくるように移動
        self.z_translate(z)
        self.z_translate(-z)

    # エンドエフェクタを指定量だけ前または後ろに動かすだけの動作
    def y_translate(self, y):
        # 一方向のみに移動。行ったら行きっぱなし
        self.move_effector_translate(0, y, 0)

    # 指定量だけ前に動かして、指定量だけ後ろに動かすだけの動作
    def y_updn(self, y):
        # 行ったら同じところに返ってくるように移動
        self.y_translate(y)
        self.y_translate(-y)

    # エンドエフェクタを指定量だけ前または後ろに動かすだけの動作
    def x_translate(self, x):
        # 一方向のみに移動。行ったら行きっぱなし
        self.move_effector_translate(x, 0, 0)

    # 指定量だけ前に動かして、指定量だけ後ろに動かすだけの動作
    def x_updn(self, x):
        # 行ったら同じところに返ってくるように移動
        self.x_translate(x)
        self.x_translate(-x)

# アームで突くためのクラス
# 狙うのが横か下かでエイムしてから突き出す動作を変更する
class arm_thrust_node():
    def __init__(self, string = "beside"):
        # 狙うのが横か下かを判定する
        # 同時にデフォルトの x y z 座標を指定
        self.besideordown = 0
        self.default_pose = []

        if string == "beside":
            self.besideordown = 0
            # 下を向くときはまずこの位置にエンドエフェクタを持っていく
            self.default_pose = [0, 0, 0.3]
        elif string == "down":
            self.besideordown = 1
            # 下を向くときはまずこの位置にエンドエフェクタを持っていく
            self.default_pose = [0.25, 0, 0.25]
        else:
            print("unknown search pose " + str(string), file=sys.strerr)
            sys.exit(1)

        # ノードの開始
        rospy.init_node("subscribe")
        # arm グループの取得
        self.arm = moveit_commander.MoveGroupCommander("arm")
        # 速度加速度の制限を設ける
        self.arm.set_max_velocity_scaling_factor(0.7)
        self.arm.set_max_acceleration_scaling_factor(1.0)
        # デフォルトポーズにする
        self.go_default()
        # 平行移動用のクラスをインスタンス化
        self.arm_liner_mover = liner_movement(self.arm)

    # デフォルトポーズにセットする
    def go_default(self):
        # サーチポーズに戻る
        if self.besideordown == 0:
            # 横向きのとき
            default_pose = [
                    self.default_pose[0],
                    self.default_pose[1],
                    self.default_pose[2],
                    math.pi / 2,
                    0,
                    - math.pi / 2
                    ]
        if self.besideordown == 1:
            # 下向きの時
            default_pose = [
                    self.default_pose[0],
                    self.default_pose[1],
                    self.default_pose[2],
                    - math.pi,
                    0,
                    math.pi
                    ]
        self.arm.set_pose_target(default_pose)
        self.arm.go()

    def aim_yz(self, difference_y, difference_z):
        self.arm_liner_mover.y_translate(difference_y)
        self.arm_liner_mover.z_translate(difference_z)

    def aim_xy(self, difference_x, difference_y):
        self.arm_liner_mover.x_translate(difference_x)
        self.arm_liner_mover.y_translate(difference_y)

    def aim(self, target_coordinate):
        # 現在の位置姿勢を取得
        current_pose = Pose()
        current_pose = self.arm.get_current_pose()
        # 現在の位置だけを取得
        current_coordinate_pose = current_pose.pose.position
        current_coordinate = [current_coordinate_pose.x, current_coordinate_pose.y, current_coordinate_pose.z]
        # エンドエフェクタから見た目標座標の位置を取得
        differences = diff(target_coordinate, current_coordinate)

        difference_x = differences[0]
        difference_y = differences[1]
        difference_z = differences[2]

        # 横向きの時
        if self.besideordown == 0:
            # 横向きのときは y - z を先に揃えて、x を突く
            # y - z を揃える
            self.aim_yz(difference_y, difference_z)
            # x 方向に突きをする
            self.arm_liner_mover.x_updn(target)

        # 下向きの時
        if self.besideordown == 1:
            # 下向きのときは x - y を先に揃えて、z を動かす
            # x - y を揃える
            self.aim_xy(difference_x, difference_y)
            # z 方向に突きをする
            self.arm_liner_mover.z_updn(difference_z)

    def main(self, target):
        self.aim(target)
        # rospy.spin()

""" ライブラリとして使用するためいらない
if __name__ == "__main__":
    # atn => Arm Thrust Node
    atn = arm_thrust_node()
    # どれだけ前に突き出すかメートル単位で指定する エンドエフェクタの到達半径は 大体 0.49*** [m] くらい
    target = 0.135
    # 実際に前に突き出す
    atn.main(- target)
    # 処理が終了したらノードを閉じる
    del atn
    # 終了コード
    sys.exit(0)
"""

