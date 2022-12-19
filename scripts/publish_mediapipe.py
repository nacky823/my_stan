#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# publish_mediapipe.py

# SPDX-FileCopyrightText: 2022 nacky823 | YAZAWA Kenichi s21c1036hn@s.chibakoudai.jp
# SPDX-License-Identifier:

# RealSense のカラーカメラは 6 ?
DEVICE_NUM = 0
# めちゃくちゃキモいけどここにないと面倒くさいからここに記述

import copy
import argparse
import rospy
import cv2 as cv
import numpy as np
import mediapipe as mp
import statistics
import math
from collections import deque
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point

diff_point = Point() # 偏差
target_point = Point()  # カメラ画像中心の座標
target_point.z = 0      # camera target z

# カメラの中心点からどのくらい下にずれているべきか
OFFSET = - 800

# FPS の表示
FPS_DEBUG = False
# Rect の使用 # これマジで意味がわからん本当にいらないと思うんだけど
RECT_DEBUG = False

# 複数の座標の重心を取得するインデックス    Yazawa
# 0 と 1 を取得するので z 座標が含まれていても問題なし
def center(points):
    mid_x = statistics.mean(points.T[0])
    mid_y = statistics.mean(points.T[1])
    return np.array([mid_x, mid_y])

# 二点間の距離を図るノルム２    Yazawa
# こっちは x, y の二次元で与えないとエラー出そう
def norm(a, b):
        return np.linalg.norm(np.subtract(a, b), ord = 2)

# xyzrpy to xyzxyzw
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

class CvFpsCalc(object):
    def __init__(self, buffer_len=1):
        self._start_tick = cv.getTickCount()
        self._freq = 1000.0 / cv.getTickFrequency()
        self._difftimes = deque(maxlen=buffer_len)

    def get(self):
        current_tick = cv.getTickCount()
        different_time = (current_tick - self._start_tick) * self._freq
        self._start_tick = current_tick

        self._difftimes.append(different_time)

        fps = 1000.0 / (sum(self._difftimes) / len(self._difftimes))
        fps_rounded = round(fps, 2)

        return fps_rounded

def get_args():
    parser = argparse.ArgumentParser()

    # ウェブカムか PC 内臓のやつかを選択できるようにデバイスを変数化    yazawa
    global DEVICE_NUM
    parser.add_argument("--device", type=int, default = DEVICE_NUM)    # Real sens video6
    parser.add_argument("--width", help='cap width', type=int, default=960)
    parser.add_argument("--height", help='cap height', type=int, default=540)

    parser.add_argument("--max_num_faces", type=int, default=1)
    parser.add_argument("--min_detection_confidence",
                        help='min_detection_confidence',
                        type=float,
                        default=0.7)
    parser.add_argument("--min_tracking_confidence",
                        help='min_tracking_confidence',
                        type=int,
                        default=0.5)

    parser.add_argument('--use_brect', action='store_true')

    args = parser.parse_args()

    return args


def main():
    global RECT_DEBUG, FPS_DEBUG, OFFSET

    rospy.init_node("publish_mediapipe")   # node 初期化
    #pub_str = rospy.Publisher("mediapipe_string", String, queue_size=10)    # publisher 作成
    pub_diff = rospy.Publisher("/mediapipe_difference", Pose, queue_size=10)    # publisher 作成


    # 引数解析 #################################################################
    args = get_args()

    cap_device = args.device
    cap_width = args.width
    cap_height = args.height

    max_num_faces = args.max_num_faces
    #print('max_num_faces = ', end='')
    #print(max_num_faces)
    min_detection_confidence = args.min_detection_confidence
    #print('min_detection_confidence = ', end='')
    #print(min_detection_confidence)
    min_tracking_confidence = args.min_tracking_confidence
    #print('min_tracking_confidence = ', end='')
    #print(min_tracking_confidence)

    if RECT_DEBUG:
        use_brect = args.use_brect 

    # カメラ準備 ###############################################################
    cap = cv.VideoCapture(cap_device)
    print('Capture_device : ', end='')
    print(cap_device)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, cap_width)
    # 640
    print('WIDTH : x_max = ', end='')
    x_max = cap.get(cv.CAP_PROP_FRAME_WIDTH)
    print(x_max)
    #print(cap.get(cv.CAP_PROP_FRAME_WIDTH))
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, cap_height)
    # 540
    print('HEIGHT : y_max = ', end='')
    y_max = cap.get(cv.CAP_PROP_FRAME_HEIGHT)
    print(y_max)

    target_point.x = (x_max / 2)
    target_point.y = (y_max / 2) - OFFSET
    target_point.z = 0.27

    # モデルロード #############################################################
    mp_face_mesh = mp.solutions.face_mesh
    face_mesh = mp_face_mesh.FaceMesh(
        max_num_faces=max_num_faces,
        min_detection_confidence=min_detection_confidence,
        min_tracking_confidence=min_tracking_confidence,
    )

    # FPS計測モジュール ########################################################
    if FPS_DEBUG:
        cvFpsCalc = CvFpsCalc(buffer_len = 10) # 10.31 ここまで終わった
    #print(cvFpsCalc.get())
    #print(cvFpsCalc.get())

    ret, image = cap.read()
    # print('各次元の長さ : (y, x, ?) =', image.shape)
    # print('全要素数 :', image.size)

    #rate = rospy.Rate(10)

    # while not rospy.is_shutdown():
    while True:
        #print(cvFpsCalc.get()) #whileないだけのようだ。↓　
        if FPS_DEBUG:
            display_fps = cvFpsCalc.get()
        #print(display_fps)
        #print(cvFpsCalc.get()) #この文を加えるたび、fpsが + display_fps される。
        #print(cvFpsCalc.get())
        #print(cvFpsCalc.get())

        # カメラキャプチャ #####################################################
        #print(cap.read())
        ret, image = cap.read() #この書き方で成立する意味がわからん。
        #print(cap.get())
        #print(ret)
        #print(image)
        if not ret:
            break
        image = cv.flip(image, 1)  # ミラー表示
        debug_image = copy.deepcopy(image)
        #print(debug_image)
        #print('/////////////////////////////////////////////////////////////////////////??')


        # 検出実施 #############################################################
        image = cv.cvtColor(image, cv.COLOR_BGR2RGB)
        results = face_mesh.process(image)

        # 口の座標が取得できる場合のみ publish したい
        mouth_exist = False

        # 描画 ################################################################
        #print(results.multi_face_landmarks)
        #print('/////////////////////////////////////////////////////////////////////////??')
        if results.multi_face_landmarks is not None:
            for face_landmarks in results.multi_face_landmarks:
                mouth_exist = True
                # 外接矩形の計算
                if RECT_DEBUG:
                    brect = calc_bounding_rect(debug_image, face_landmarks)
                # 描画
                debug_image = draw_landmarks(debug_image, face_landmarks)
                if RECT_DEBUG:
                    debug_image = draw_bounding_rect(use_brect, debug_image, brect)

        # 照準点を描画したいんだけど...
        # aim_sight = np.array([int(target_point.x), int(target_point.y)])
        # print("aim_sight = ", end = "")
        # print(aim_sight)
        # cv.circle(image, aim_sight, 2, (255, 255, 0), 2)

        if FPS_DEBUG:
            cv.putText(debug_image, "FPS:" + str(display_fps), (10, 30),
                       cv.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2, cv.LINE_AA)

        # キー処理(ESC：終了) #################################################
        key = cv.waitKey(1)
        if key == 27:  # ESC
            break

        # 画面反映 #############################################################
        cv.imshow('MediaPipe Face Mesh Demo', debug_image)

        # 1 秒ごとにトピックを送信
        #rate = rospy.Rate(1)
        #while not rospy.is_shutdown():

        # トピックを送信
        #msg_str = "Publishing {}".format(rospy.get_time())
        #pub_str.publish(msg_str)

        # これ無いとだめじゃね？        yazawa
        global diff_point
        if mouth_exist:
            pub_list = [diff_point.x, diff_point.y, diff_point.z, math.pi / 2,0,-math.pi / 2]
            pub_diff_pose = list2pose(pub_list)
            pub_diff.publish(pub_diff_pose)

            #rospy.loginfo("Message '{}' published".format(msg_str))
            rospy.loginfo(pub_diff_pose)

            # 1 秒スリープする
            #rate.sleep()
            rospy.sleep(1) # 0.05

        else:
            print("lost ...")

        rospy.sleep(0.02)

    cap.release()
    cv.destroyAllWindows()

def calc_bounding_rect(image, landmarks):
    image_width, image_height = image.shape[1], image.shape[0]
    #print('image.shape =', image.shape)
    #print('/////////////////////////////////////////////////////////////////////////??')

    landmark_array = np.empty((0, 2), int)

    for _, landmark in enumerate(landmarks.landmark):
        #print(landmarks.landmark)
        #print('/////////////////////////////////////////////////////////////////////////??')
        landmark_x = min(int(landmark.x * image_width), image_width - 1)
        landmark_y = min(int(landmark.y * image_height), image_height - 1)


        landmark_point = [np.array((landmark_x, landmark_y))]

        landmark_array = np.append(landmark_array, landmark_point, axis=0)

    x, y, w, h = cv.boundingRect(landmark_array)

    return [x, y, x + w, y + h]


def draw_landmarks(image, landmarks):
    image_width, image_height = image.shape[1], image.shape[0]

    landmark_point = []
    landmark_point_mouth = []
    landmark_point_left_eye = []
    landmark_point_right_eye = []

    for index, landmark in enumerate(landmarks.landmark):
        if landmark.visibility < 0 or landmark.presence < 0:
            continue

        landmark_x = min(int(landmark.x * image_width), image_width - 1)
        landmark_y = min(int(landmark.y * image_height), image_height - 1)
        landmark_z = landmark.z

        if 13 <= index and index < 15:
            landmark_point_mouth.append((landmark_x, landmark_y))
        if index == 133 or index == 246:
            landmark_point_left_eye.append((landmark_x, landmark_y))
        if index == 362 or index == 466:
            landmark_point_right_eye.append((landmark_x, landmark_y))

    # 口の中心点を取得
    mouth_points = np.array([landmark_point_mouth[0], landmark_point_mouth[1]])
    target = center(mouth_points)

    ### 深さの計算 Yazawa ###
    # 両目の特徴点の座標を取得
    left_eye_points = np.array([landmark_point_left_eye[0], landmark_point_left_eye[1]])
    right_eye_points = np.array([landmark_point_right_eye[0], landmark_point_right_eye[1]])
    # 両目の中心点を取得
    left_eye = center(left_eye_points)
    # print(left_eye)
    right_eye = center(right_eye_points)
    # print(right_eye)
    # 両目の距離を計算
    px = norm(left_eye, right_eye)
    # 三角法で距離を計算    単位 [m]
    dist1 = 50     # px
    diff1 = 0.94      # m
    dist2 = 150      # px
    diff2 = 0.27     # m
    face_depth = (px - dist1) * (diff1 - diff2) / (dist1 - dist2) + diff1

    current_point = Point() # 口中心の座標
    current_point.x = target[0]
    current_point.y = target[1]
    # とりま決め打ちで
    current_point.z = 0.2
    # current_point.z = face_depth # 深さ情報の追加単位は [m] Yazawa

    print("current_point : ", end = "")
    print(current_point)
    print("target_point : ", end = "")
    print(target_point)
    global diff_point
    diff_point.y = ( target_point.x - current_point.x ) / 2000
    diff_point.z = ( target_point.y - current_point.y ) / 2000
    diff_point.x = ( target_point.z - current_point.z )
    print("diff_point : ", end = "")
    print(diff_point)

    cv.circle(image, target, 3, (255, 0, 255), 2) # 口中心の描写

    #cv.putText(image, "z:" + str(round(landmark_z, 3)), (landmark_x - 10, landmark_y - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv.LINE_AA)

    # endif len(landmark_point)

    return image


def draw_bounding_rect(use_brect, image, brect):
    if use_brect:
        # 外接矩形
        cv.rectangle(image, (brect[0], brect[1]), (brect[2], brect[3]),
                     (0, 255, 0), 2)

    return image

if __name__ == '__main__':
    print(diff_point)
    main()
