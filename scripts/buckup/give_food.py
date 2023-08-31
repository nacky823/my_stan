#!/usr/bin/env python3
# coding: utf-8
# thrust.py

# SPDX-FileCopyrightText: 2022 nacky823 | YAZAWA Kenichi s21c1036hn@s.chibakoudai.jp
# SPDX-License-Identifier: MIT License

import rospy
from geometry_msgs.msg import Point,Pose
import math

def list2pose(_list):
    _pose = Pose()
    _pose.position.x = _list[0]
    _pose.position.y = _list[1]
    _pose.position.z = _list[2]
    sp = math.sin(_list[3] / 2)
    cp = math.cos(_list[3] / 2)
    st = math.sin(_list[4] / 2)
    ct = math.cos(_list[4] / 2)
    sk = math.sin(_list[5] / 2)
    ck = math.cos(_list[5] / 2)
    _pose.orientation.w = cp * ct * ck + sp * st * sk
    _pose.orientation.x = sp * ct * ck - cp * st * sk
    _pose.orientation.y = cp * st * ck + sp * ct * sk
    _pose.orientation.z = cp * ct * sk - sp * st * ck
    return _pose

class give_food_node():
    def __init__(self):
        # atn => Arm Thrust Node
        self.generate_publisher()
        # publish するポーズを指定
        publish_pose_list = [
                0.18,
                0,
                0.18,
                - math.pi,
                0,
                math.pi / 2
                ]
        # pose 型に変換
        self.publish_pose = list2pose(publish_pose_list)

    # パブリッシャを作成する
    def generate_publisher(self):
        print("topic name : /food_coordinate")
        self.pub = rospy.Publisher("/food_coordinate", Pose, queue_size = 16)

    def publish(self):
        print("publish : ", end = "")
        print(self.publish_pose)
        self.pub.publish(self.publish_pose)
    
def main():
    rospy.init_node("give_food_node")
    gfn = give_food_node()
    rate = rospy.Rate(1)
    print("while ... ")
    while not rospy.is_shutdown():
        gfn.publish()
        rate.sleep()
    print("break ... ")

if __name__ == "__main__":
    main()

