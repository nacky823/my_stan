#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def callback(msg):
    rospy.loginfo("sub_test_Message '{}' recieved".format(msg.data))

def subscriber():

    rospy.init_node("sub_mp")   # ノードを初期化

    rospy.Subscriber("medi", String, callback)     # 受信者を作成
    
    rospy.spin()    # ノードが終了するまで待機

if __name__ == "__main__":
    subscriber()
