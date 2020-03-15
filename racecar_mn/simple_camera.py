#!/usr/local/bin/python3

# node for naively and publishing rgb picture from RealSense

import rclpy
from sensor_msgs.msg import Image

import cv2
import numpy as np

# connect to camera
cap = cv2.VideoCapture(2)

# init ROS
rclpy.init()
rclpy.create_node('camera')
pub_im = rclpy.create_publisher(Image,'/camera', queue_size=1)

# main body loop
while not rclpy.is_shutdown():
    # capture frame-by-frame
    _, frame = cap.read()

    # confirm frame is present
    if frame is None:
        continue

    # build Image message
    msg = Image()
    msg.height = frame.shape[0]
    msg.width = frame.shape[1]
    msg.encoding = 'bgr8'
    msg.is_bigendian = 0
    msg.step = 3 * msg.width
    msg.data = frame.flatten().tostring()

    # publish Image message
    pub_im.publish(msg)

# free resources
cap.release()


#rclpy.spin() # not necessary?