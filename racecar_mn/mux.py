#!/usr/local/bin/python3

# node for arbitrating who has drive control

# stdlib imports
import sys

# python 3
import rclpy
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSProfile

from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped

from enum import Enum, auto

class MuxMode(Enum):
    """ Denotes which messages pass through the MUX """

    IDLE = (0, None)            # nothing passes through
    GAMEPAD = (auto(), 4)       # gamepad commands pass through
    AUTONOMY = (auto(), 5)      # drive commands pass through
    ERROR = (auto(), None)      # ignore this value
    # note: 'auto()' requires python 3.6+

    def __init__(self, value, enable_button):
        self.enable = None
        self._value_ = value 
        self.enable = enable_button

# # If we want to configure these from parameters, do so here...
# MuxMode.GAMEPAD.enable=4
# MuxMode.AUTONOMY.enable=5

# list all file-global variables
_mode = MuxMode.IDLE
_node = None
_pub = None

# gamepad callback
def joy_callback(msg):
    global _mode, _pub

    # if LB is pressed, enable teleop
    if msg.buttons[MuxMode.GAMEPAD.enable] == 1:
        _mode = MuxMode.GAMEPAD
    # if RB is pressed, enable autonomy
    elif msg.buttons[MuxMode.AUTONOMY.enable] == 1:
        _mode = MuxMode.AUTONOMY

    # otherwise default, publish stop
    else:
        _mode = MuxMode.IDLE
        _pub.publish(AckermannDriveStamped())


# callback for gamepad_drive topic
def gamepad_drive_callback(msg):
    global _mode, _pub

    if _mode == MuxMode.GAMEPAD:
        _pub.publish(msg)

# callback for drive topic
def autonomy_drive_callback(msg):
    global _mode, _pub

    if _mode == MuxMode.AUTONOMY:
        _pub.publish(msg)


def main(args=None):
    global _node, _pub
        
    # init ROS
    rclpy.init(args=args)

    _node = rclpy.create_node('mux_node')

    # see example: https://github.com/ros2/demos/blob/master/topic_monitor/topic_monitor/scripts/data_publisher.py
    qos_profile = QoSProfile(depth=1)
    qos_profile.history = QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST
    qos_profile.reliability = QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
    qos_profile.durability = QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE

    _pub = _node.create_publisher(AckermannDriveStamped, '/mux_out', qos_profile)

    _node.create_subscription(AckermannDriveStamped, '/gamepad_drive', gamepad_drive_callback, qos_profile)
    _node.create_subscription(AckermannDriveStamped, '/drive', autonomy_drive_callback, qos_profile)
    _node.create_subscription(Joy, '/joy', joy_callback, qos_profile)
    

    # wait before shutdown
    rclpy.spin(_node)

    _node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main(sys.argv)
