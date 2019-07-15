#!/usr/bin/python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist 

# get param file values
vel_scale = rospy.get_param('gamepad/x_vel_scale')
turn_scale = rospy.get_param('gamepad/y_turn_scale')

# gamepad callback
def joy_callback(msg):
    global drive_pub, xscale, yscale
    drive_msg =  Twist()
    if msg.buttons[4] == 1:
            drive_msg.angular.z = msg.axes[3] * vel_scale
            drive_msg.linear.x = msg.axes[1] * turn_scale
    drive_pub.publish(drive_msg)

# init ROS
rospy.init_node("gamepad")
drive_pub = rospy.Publisher("/drive", Twist, queue_size = 10)
joy_sub = rospy.Subscriber("/joy", Joy, joy_callback)

# wait before shutdown
rospy.spin()
