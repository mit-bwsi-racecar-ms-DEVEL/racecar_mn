#!/usr/bin/python

# Abstraction layer for controlling the motors

import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSProfile
from . import maestro
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String
import time

PWM_SPEED_MAX = 9000
PWM_SPEED_MIN = 3000
PWM_SPEED_CHANNEL = 0

PWM_TURN_LEFT = 9000
PWM_TURN_RIGHT = 3000
PWM_TURN_LEFT_CAP = 9000
PWM_TURN_RIGHT_CAP = 3000
PWM_TURN_CHANNEL = 1

def main(args=None):
    rclpy.init()
    node = rclpy.create_node('pwm')
    
    try:  
        qos_profile = QoSProfile(depth=1)
        qos_profile.history = QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST
        qos_profile.reliability = QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE
        pub = node.create_publisher(String, "/pwm_debug", qos_profile = 1)
    
        # initialize servo controller
        controller = maestro.Controller()
    
        def drive_callback(msg):
            """
            Callback function passed to the ROS subscriber /drive to compute signals to send to the PWM
            """
        
            # Set drive speed
            controller.setTarget(PWM_SPEED_CHANNEL, int(msg.drive.speed))

            # Set drive angle
            controller.setTarget(PWM_TURN_CHANNEL, int(msg.drive.steering_angle))
        
            my_msg = String()
            my_msg.data =  ("Velocity Signal: " + str(msg.drive.speed) + "|" \
                         "Turn Signal: " + str(msg.drive.steering_angle))
            pub.publish(my_msg)

        node.create_subscription(AckermannDriveStamped, '/drive', drive_callback, qos_profile)
    
        # speed config
        controller.setRange(PWM_SPEED_CHANNEL, PWM_SPEED_MIN, PWM_SPEED_MAX)
        controller.setSpeed(PWM_SPEED_CHANNEL, 0)
        controller.setAccel(PWM_SPEED_CHANNEL, 0)
        controller.setTarget(PWM_SPEED_CHANNEL, (PWM_SPEED_MAX + PWM_SPEED_MIN) // 2)

        # turning config
        controller.setRange(PWM_TURN_CHANNEL, PWM_TURN_RIGHT, PWM_TURN_LEFT)
        controller.setSpeed(PWM_TURN_CHANNEL, 0)
        controller.setAccel(PWM_TURN_CHANNEL, 0)
        controller.setTarget(PWM_TURN_CHANNEL, (PWM_TURN_RIGHT + PWM_TURN_LEFT) // 2)
        
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    finally:
        controller.close()
        rclpy.shutdown()
if __name__ == '__main__':
    main(sys.argv)
