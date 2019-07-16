#!/usr/bin/python

# node abstraction for using maestro.py driver for servo/motor

import rospy
import maestro
from ackermann_msgs.msg import AckermannDriveStamped
import time

Y_OFFSET = rospy.get_param('steering_y_offset')

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
    global controller, Y_OFFSET
    lin_vel = map_val(msg.drive.speed, -1.5, 1.5, 3000, 9000)
    turn_angle = map_val(msg.drive.steering_angle, -0.4, 0.4, 8000, 4000) 

    # set drive speed
    controller.setTarget(0, int(lin_vel))

    # added Y_OFFSET to center servo
    controller.setTarget(1, int(turn_angle+Y_OFFSET)) 

# init ros
rospy.init_node('pwm')
rospy.Subscriber('/motor', AckermannDriveStamped, motor_callback)

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

# wait and shutdown
rospy.spin()
controller.close()
