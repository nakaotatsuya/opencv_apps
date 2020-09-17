#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import rospy
import message_filters
from sensor_msgs.msg import Image
import numpy as np
from PIL import Image as _Image

from cv_bridge import CvBridge, CvBridgeError
import os
import cv2 as cv

class PileUp(object):
    def __init__(self):
        self.img_sub = rospy.Subscriber("image", Image, self._callback, queue_size=1)
        self.pub = rospy.Publisher("~output", Image, queue_size=1)

        self.bridge = CvBridge()

    def _callback(self, img_msg):
        try:
            cv_background_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        now_dir = os.path.dirname(os.path.abspath(__file__))
        cv_overlay_image = cv.imread(
            os.path.join(now_dir, "../picture/face_icon.jpg"), cv.IMREAD_UNCHANGED)

        cv_background_image = cv.resize(cv_background_image, (480,320))
        cv_overlay_image = cv.resize(cv_overlay_image, (240,160))

        #print(cv_background_image.shape)
        #print(cv_overlay_image.shape)
        

        #height = cv_overlay_image.shape[0]
        #width = cv_overlay_image.shape[1]
        #cv_overlay_image = cv.resize(cv_overlay_image, (int(width*0.5),int(height*0.5)))

        #point = (120,120)

        #added_image = cv.addWeighted(cv_background_image,0.7,cv_overlay_image,0.2,0)
        
        # OpenCV形式の画像をPIL形式に変換(α値含む)
        # 背景画像
        cv_rgb_bg_image = cv.cvtColor(cv_background_image, cv.COLOR_BGR2RGBA)
        pil_rgb_bg_image = _Image.fromarray(cv_rgb_bg_image)
        pil_rgba_bg_image = pil_rgb_bg_image.convert('RGBA')
        # オーバーレイ画像
        cv_rgb_ol_image = cv.cvtColor(cv_overlay_image, cv.COLOR_BGRA2RGBA)
        pil_rgb_ol_image = _Image.fromarray(cv_rgb_ol_image)
        pil_rgba_ol_image = pil_rgb_ol_image.convert('RGBA')

        # composite()は同サイズ画像同士が必須のため、合成用画像を用意
        pil_rgba_bg_temp = _Image.new('RGBA', pil_rgba_bg_image.size,
                                     (255, 255, 255, 170))

        point = (120,80)
        
        # 座標を指定し重ね合わせる
        #pil_rgba_bg_iamge.putalpha(128)
        pil_rgba_ol_image.putalpha(128)
        
        pil_rgba_bg_image.paste(pil_rgba_ol_image, point, pil_rgba_ol_image)
        
        #result_image = \
        #    _Image.composite(pil_rgba_bg_image, pil_rgba_ol_image, pil_rgba_bg_temp)

        

        # OpenCV形式画像へ変換
        #cv_bgr_result_image = cv.cvtColor(np.asarray(result_image), cv.COLOR_RGBA2BGRA)
        cv_bgr_result_image = cv.cvtColor(np.asarray(pil_rgba_bg_image), cv.COLOR_RGBA2BGRA)

        circle_pos = [(240,30), (240,290), (110,160), (370,160)]
        for xy in circle_pos:
            cv_bgr_result_image = cv.circle(cv_bgr_result_image, xy, 15, (0,0,255), thickness=3)
        
        
        pub_msg = self.bridge.cv2_to_imgmsg(cv_bgr_result_image, "bgra8")
        pub_msg.header = img_msg.header
        #pub_msg = self.bridge.cv2_to_imgmsg(added_image, "bgr8")
        try:
            self.pub.publish(pub_msg)
        except CvBridgeError as e:
            print(e)

if __name__ == "__main__":
    rospy.init_node("pile_up", anonymous=True)
    pu = PileUp()
    rospy.spin()
