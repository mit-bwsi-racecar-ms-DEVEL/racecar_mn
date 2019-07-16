#!/usr/bin/python

# node for ensuring commanded speed does not exceed throttle

import rospy
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped

# get param file values
VEL_MAX = rospy.get_param('throttle_vel_max')

# callback that throttles max speed
def drive_callback(msg):
    global motor_pub
    if msg.drive.speed > VEL_MAX:
        msg.drive.speed = VEL_MAX
    motor_pub.publish(msg)

# init ROS
rospy.init_node('throttle')
motor_pub = rospy.Publisher('/motor', AckermannDriveStamped, queue_size=1)
rospy.Subscriber('/drive', AckermannDriveStamped, drive_callback)

# wait before shutdown
rospy.spin()
