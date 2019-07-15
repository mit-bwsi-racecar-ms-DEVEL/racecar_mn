#!/usr/bin/env python

# node abstraction for servo/motor

import rospy
import maestro
from geometry_msgs.msg import Twist
import time

# simple function to do a linear mapping
def map_val(value, inMin, inMax, outMin, outMax):
    inSpan = inMax - inMin
    outSpan = outMax - outMin
    valScaled = float(value - inMin) / float(inSpan)
    return outMin + (valScaled * outSpan)

# pass this Twist msg.linear.x for speed and msg.angular.z for turn
# x [-1.5, 1.5] -> | -1.5 full reverse | 1.5 full forward |
# z [-0.4, 0.4] -> | -0.4 full left    | 0.4 full right   |
def drive_callback(msg):
    global controller
    lin_vel = map_val(msg.linear.x,-1.5,1.5,3000,9000)

    turn_angle = map_val(msg.angular.z,-0.4,0.4,8500,3500) 
    controller.setTarget(0,int(lin_vel))

    # added 250 to center servo
    controller.setTarget(1,int(turn_angle-250)) 

# init ros
rospy.init_node('pwm')
rospy.Subscriber("/drive", Twist, drive_callback)

# default config
controller = maestro.Controller()
controller.setRange(0,3000,9000)
controller.setRange(1,3000,9000)
controller.setSpeed(0,0)
controller.setSpeed(1,0)
controller.setAccel(0,0)
controller.setAccel(1,0)     
controller.setTarget(0,6000)
controller.setTarget(1,6000)

# wait and shutdown
rospy.spin()
controller.close()
