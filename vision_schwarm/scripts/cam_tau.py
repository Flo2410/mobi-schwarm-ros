#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError

rospy.init_node('cam_tau', anonymous=False)

VideoRaw = rospy.Publisher('/VideoRaw', Image, queue_size=10)

cam = cv2.VideoCapture(0)

factor = 1

cam.set(3,160*factor)
cam.set(4,120*factor)

rate = rospy.Rate(20)

while not rospy.is_shutdown():
    meta, frame = cam.read()

    msg_frame = CvBridge().cv2_to_imgmsg(frame, "bgr8")
    VideoRaw.publish(msg_frame)
    #cv2.imshow('kek', frame)

    rate.sleep()