#!/usr/bin/python

import rospy
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped

# get param file values
vel_scale = rospy.get_param('gamepad/x_vel_scale')
turn_scale = rospy.get_param('gamepad/y_turn_scale')

# gamepad callback
def joy_callback(msg):
    global drive_pub, vel_scale, turn_scale
    drive_msg = AckermannDriveStamped()
    if msg.buttons[4] == 1:
        drive_msg.drive.speed = msg.axes[1] * vel_scale
        drive_msg.drive.steering_angle = msg.axes[3] * turn_scale
    drive_pub.publish(drive_msg)

# init ROS
rospy.init_node("gamepad")
drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size = 1)
joy_sub = rospy.Subscriber("/joy", Joy, joy_callback)

# wait before shutdown
rospy.spin()
