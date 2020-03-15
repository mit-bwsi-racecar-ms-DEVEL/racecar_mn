#!/usr/local/bin/python3

# node for ensuring commanded speed does not exceed throttle
import rclpy

from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped

# get param file values
CAR_THROTTLE_FORWARD = rclpy.parameter.Parameter('car_throttle_forward',type_=DOUBLE)
CAR_THROTTLE_BACKWARD = rclpy.param.Paramter('car_throttle_backward',type_=DOUBLE)
CAR_THROTTLE_TURN = rclpy.param.Parameter('car_throttle_turn',type_=DOUBLE)

# callback that throttles max speed and angle
def drive_callback(msg):
    global motor_pub
    if msg.drive.speed > CAR_THROTTLE_FORWARD:
        msg.drive.speed = CAR_THROTTLE_FORWARD

    if msg.drive.speed < -CAR_THROTTLE_BACKWARD:
        msg.drive.speed = -CAR_THROTTLE_BACKWARD

    if msg.drive.steering_angle > CAR_THROTTLE_TURN:
        msg.drive.steering_angle = CAR_THROTTLE_TURN

    if msg.drive.steering_angle < -CAR_THROTTLE_TURN:
        msg.drive.steering_angle = -CAR_THROTTLE_TURN 

    motor_pub.publish(msg)

# init ROS
rclpy.init()
node=rclpy.create_node('throttle')

motor_pub = rclpy.create_publisher(AckermannDriveStamped, '/motor', queue_size=1)

mux_sub = rclpy.create_subscription(AckermannDriveStamped, '/mux_out', drive_callback)


# wait before shutdown
rclpy.spin()
