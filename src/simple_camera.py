#!/usr/bin/python

# node for naively and publishing rgb picture from RealSense

import rospy
from sensor_msgs.msg import Image

import cv2
import numpy as np

# connect to camera
cap = cv2.VideoCapture(2)

# init ROS
rospy.init_node('camera')
pub_im = rospy.Publisher('/camera',
                         Image,
                         queue_size=1)

# main body loop
while not rospy.is_shutdown():
    # capture frame-by-frame
    _, frame = cap.read()

    # confirm frame is present
    if frame is None:
        continue

    # build Image message
    msg = Image()
    msg.height = frame.shape[0]
    msg.width = frame.shape[1]
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = 3 * msg.width
    msg.data = frame.flatten().tostring()

    # publish Image message
    pub_im.publish(msg)

# free resources
cap.release()
