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

class give_food_node():
    def __init__(self):
        # atn => Arm Thrust Node
        self.generate_publiser()

    # パブリッシャを作成する
    def generate_publisher(self):
        print("topic name : /food_coordinate")
        self.pub = rospy.publisher("/food_coordinate", Pose, queue_size = 16)

    def pub(self):
        self.pub = 

if __name__ == "__main__":
    rospy.init_node("give_food_node")
    gfn = give_food_node()

