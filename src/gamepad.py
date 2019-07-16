#!/usr/bin/python

# node for turning gamepad inputs into drive commands

import rospy
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped

# get param file values
CAR_THROTTLE_FORWARD = rospy.get_param('car_throttle_forward')
CAR_THROTTLE_BACKWARD = rospy.get_param('car_throttle_backward')
CAR_THROTTLE_TURN = rospy.get_param('car_throttle_turn')
GAMEPAD_THROTTLE_SPEED_SCALE = rospy.get_param('gamepad_throttle_speed_scale')

# scale x-stick input [-1, 1] to throttled drive speed output
def scale_x(x_in):
    if x_in >= 0:
        return x_in * CAR_THROTTLE_FORWARD * GAMEPAD_THROTTLE_SPEED_SCALE
    else:
        return x_in * CAR_THROTTLE_BACKWARD * GAMEPAD_THROTTLE_SPEED_SCALE

# scale y-stick input [-1, 1] to throttled drive angle output
def scale_y(y_in):
    return y_in * CAR_THROTTLE_TURN

# gamepad callback
def joy_callback(msg):
    global drive_pub, X_SCALE, Y_SCALE
    drive_msg = AckermannDriveStamped()

    # if LB is pressed (enabling teleop mode), send drive messages
    if msg.buttons[4] == 1:
        drive_msg.drive.speed = scale_x(msg.axes[1])
        drive_msg.drive.steering_angle = scale_y(msg.axes[3])
    drive_pub.publish(drive_msg)

# init ROS
rospy.init_node('gamepad')
drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)
joy_sub = rospy.Subscriber('/joy', Joy, joy_callback)

# wait before shutdown
rospy.spin()
