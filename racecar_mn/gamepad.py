#!/usr/local/bin/python3

# node for turning gamepad inputs into drive commands

import rclpy
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped

# get param file values
CAR_THROTTLE_FORWARD = rclpy.parameter.Parameter('car_throttle_forward',type_=DOUBLE)
CAR_THROTTLE_BACKWARD = rclpy.parameter.Parameter('car_throttle_backward',type_=DOUBLE)
CAR_THROTTLE_TURN = rclpy.parameter.Parameter('car_throttle_turn',type_=DOUBLE)
GAMEPAD_THROTTLE_SPEED_SCALE = rclpy.parameter.Parameter('gamepad_throttle_speed_scale',type_=DOUBLE)
GAMEPAD_X_AXIS = rclpy.parameter.Parameter('gamepad_x_axis',type_=DOUBLE)
GAMEPAD_Y_AXIS = rclpy.parameter.Parameter('gamepad_y_axis',type_=DOUBLE)

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
    drive_msg.drive.speed = scale_x(msg.axes[GAMEPAD_X_AXIS])
    drive_msg.drive.steering_angle = scale_y(msg.axes[GAMEPAD_Y_AXIS])
    drive_pub.publish(drive_msg)

# init ROS
rclpy.init()
node = rclpy.create_node('gamepad')
drive_pub = rclpy.create_publisher(AckermannDriveStamped, '/gamepad_drive', queue_size=1)
sub = rclpy.create_subscriber(Joy, '/joy', joy_callback)

# wait before shutdown
rclpy.spin()
