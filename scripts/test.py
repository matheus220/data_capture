#!/usr/bin/env python
from __future__ import print_function

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from mongodb_bridge import MongoBridge
import time

import rospy
rospy.init_node('test', anonymous=True)

mb = MongoBridge()

bridge = CvBridge()
t1 = time.time()
image1 = mb.send_query_and_wait(SELECT=['data.image'], FROM=['camera'], WHERE=lambda item_name: u'camera1' in item_name, ORDER_BY='data.timestamp')
print(len(image1))
image2 = mb.send_query_and_wait(SELECT=['data.image'], FROM=['camera'], WHERE=lambda item_name: u'camera2' in item_name, ORDER_BY='data.timestamp')
print(len(image2))
image3 = mb.send_query_and_wait(SELECT=['data.image'], FROM=['camera'], WHERE=lambda item_name: u'camera3' in item_name, ORDER_BY='data.timestamp')
print(len(image3))

t2 = time.time()
print('TIME : ', t2-t1)

for i in range(len(image1)):
    img1 = image1[i]['data.image'][0]
    img2 = image2[i]['data.image'][0]
    img3 = image3[i]['data.image'][0]
    cv2.imshow("Camera 1", bridge.imgmsg_to_cv2(img1, "bgr8"))
    cv2.imshow("Camera 2", bridge.imgmsg_to_cv2(img2, "bgr8"))
    cv2.imshow("Camera 3", bridge.imgmsg_to_cv2(img3, "bgr8"))
    cv2.waitKey(5000)

cv2.destroyWindow("Camera 1")
cv2.destroyWindow("Camera 2")
cv2.destroyWindow("Camera 3")

rospy.spin()