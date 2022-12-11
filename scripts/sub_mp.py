#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point

def callback(msg):
    rospy.loginfo("Subscribing '{}' recieved".format(msg.data))

def subscriber():

    rospy.init_node("sub_mp")   # ノードを初期化

    rospy.Subscriber("mediapipe_str", String, callback)     # 受信者を作成
    
    rospy.spin()    # ノードが終了するまで待機

if __name__ == "__main__":
    subscriber()
