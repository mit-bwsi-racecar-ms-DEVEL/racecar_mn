#!/usr/local/bin/python3

# node abstraction for using maestro.py driver for servo/motor

import rclpy

import maestro
from ackermann_msgs.msg import AckermannDriveStamped
import time

# default config
controller = maestro.Controller()

# setup motor config
controller.setRange(0,3000,9000)
controller.setSpeed(0,0)
controller.setAccel(0,0)
controller.setTarget(0,6000)

# setup steering config
controller.setRange(1,3000,9000)
controller.setSpeed(1,0)
controller.setAccel(1,0)     
controller.setTarget(1,6000)

CAR_MAX_FORWARD = rclpy.get_param('car_max_forward')
CAR_MAX_BACKWARD = rclpy.get_param('car_max_backward')
CAR_MAX_TURN = rclpy.get_param('car_max_turn')
STEERING_CENTER_REL_OFFSET = rclpy.get_param('steering_center_rel_offset')

# simple function to do a linear mapping
def map_val(value, inMin, inMax, outMin, outMax):
    inSpan = inMax - inMin
    outSpan = outMax - outMin
    valScaled = float(value - inMin) / float(inSpan)
    return outMin + (valScaled * outSpan)

# msg.drive.speed:
# [-1.5, 1.5] -> | -1.5 full reverse | 1.5 full forward |
# msg.drive.steering_angle:
# [-0.4, 0.4] -> | -0.4 full left    | 0.4 full right   |
def motor_callback(msg):
    global CAR_MAX_FORWARD, CAR_MAX_BACKWARD, \
           CAR_MAX_TURN, STEERING_CENTER_REL_OFFSET

    if msg.drive.speed >= 0:
        lin_vel = map_val(msg.drive.speed, 0, CAR_MAX_FORWARD, 6000, 9000) 
    else:
        lin_vel = map_val(msg.drive.speed, -CAR_MAX_BACKWARD, 0, 3000, 6000)
    turn_angle = map_val(msg.drive.steering_angle,
                         -CAR_MAX_TURN, CAR_MAX_TURN, 8000, 4000) 

    # set drive speed
    controller.setTarget(0, int(lin_vel))

    # added Y_OFFSET to center servo
    controller.setTarget(1, int(turn_angle + STEERING_CENTER_REL_OFFSET))

# init ros
rclpy.init()
node = rclpy.create_node('pwm')
rclpy.create_subscriber(AckermannDriveStamped, '/motor', motor_callback)

# wait and shutdown
rclpy.spin(node)
controller.close()
