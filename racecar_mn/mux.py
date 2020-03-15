#!/usr/local/bin/python3

# node for arbitrating who has drive control

# python 3
import rclpy
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped

import enum
class MuxMode(enum.Enum):
    """ Denotes which messages pass through the MUX """
    NOTHING = ''           # nothing passes through
    GAMEPAD = 'gamepad'    # gamepad commands pass through
    AUTONOMY = 'autonomy'  # drive commands pass through
    OTHER = ''             # ignore this value

# list all file-global variables
mux_mode = ''
mux_out_pub = None

# gamepad callback
def joy_callback(msg):
    global mux_mode

    # if LB is pressed, enable teleop
    if msg.buttons[4] == 1:
        mux_mode = 'gamepad'
    # if RB is pressed, enable autonomy
    elif msg.buttons[5] == 1:
        mux_mode = 'autonomy'
    # otherwise default, publish stop
    else:
        mux_mode = ''
        mux_out_pub.publish(AckermannDriveStamped())


# callback for gamepad_drive topic
def gamepad_drive_callback(msg):
    global mux_mode, mux_out_pub
    if mux_mode == 'gamepad':
        mux_out_pub.publish(msg)

# callback for drive topic
def drive_callback(msg):
    global mux_mode, mux_out_pub
    if mux_mode == 'autonomy':
        mux_out_pub.publish(msg)

# init ROS
# rclpy.init_node('mux') # python2
rclpy.init(args=sys.argv)
node = rclpy.create_node('mux')

# # python2
# mux_out_pub = rclpy.Publisher('/mux_out', AckermannDriveStamped, queue_size=1)

# python3
mux_out_pub = node.create_publisher( AckermannDriveStamped, '/mux_out', queue_size=1)

# # python2
# rclpy.Subscriber('/gamepad_drive', AckermannDriveStamped, gamepad_drive_callback)
# rclpy.Subscriber('/drive', AckermannDriveStamped, drive_callback)
# rclpy.Subscriber('/joy', Joy, joy_callback)


sub_gamepad = node.create_subscription(AckermannDriveStamped, '/gamepad_drive', gamepad_drive_callback)
sub_drive = node.create_subscription(AckermannDriveStamped, '/drive', drive_callback)
sub_joy = node.create_subscription(Joy, '/joy', joy_callback)


# wait before shutdown
rclpy.spin(node)
