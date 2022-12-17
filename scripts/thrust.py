#!/usr/bin/env python3
# coding: utf-8
# thrust.py

import rospy
import moveit_commander
from geometry_msgs.msg import Point,Pose
from tf.transformations import quaternion_from_euler
import math
import numpy as np
import sys

# 二点間の距離を求める
def norm(a, b):
    return np.linalg.norm(np.subtract(a, b), ord = 2)

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
    ### 同じ要領で x, y 方向にも作ると便利かもしれない ###

# アームで突くためのクラス
class arm_thrust_node():
    def __init__(self):
        # ノードの開始
        rospy.init_node("subscribe")
        # arm グループの取得
        self.arm = moveit_commander.MoveGroupCommander("arm")
        # 速度加速度の制限を設ける
        self.arm.set_max_velocity_scaling_factor(0.7)
        self.arm.set_max_acceleration_scaling_factor(1.0)
        # "vertical" に一旦戻す
        self.arm.set_named_target("vertical")
        self.arm.go()
        # サーチポーズにする
        self.go_search()
        # 平行移動用のクラスをインスタンス化
        self.arm_liner_mover = liner_movement(self.arm)

    # サーチポーズにセットする
    def go_search(self):
        # サーチポーズに戻る
        search_pose = [
                0,
                0,
                0.3,
                math.pi / 2,
                0,
                - math.pi / 2
                ]
        self.arm.set_pose_target(search_pose)
        self.arm.go()

    def main(self, target):
        self.arm_liner_mover.x_updn(target)
        # rospy.spin()

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

