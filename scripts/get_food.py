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

if __name__ == "__main__":
    # atn => Arm Thrust Node
    atn = arm_thrust_node("down")  # 横に突き出す場合は "beside" を指定（見するとエラーを吐いて終了）
    # どれだけ前に突き出すかメートル単位で指定する エンドエフェクタの到達半径は 大体 0.49*** [m] くらい
    target = [0.2, 0.0, 0.28]
    # 実際に下に突き出す
    atn.main(target)
    # 処理が終了したらノードを閉じる
    del atn
    # 終了コード
    sys.exit(0)

