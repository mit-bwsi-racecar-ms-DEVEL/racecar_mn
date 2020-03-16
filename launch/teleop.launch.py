#!/usr/bin/python3

import launch
import launch.actions
import launch.substitutions
from launch_ros.actions import Node


def generate_launch_description():
    param_file_path = '/home/racecar/racecar_ws/base_ws/src/racecar_mn/config/params.yaml'
    
    return launch.LaunchDescription([
	Node(
	    package='joy',
            node_executable='joy_node',
            node_name='joy_node',
	),
        Node(
	    package='racecar_mn',
            node_executable='gamepad',
            node_name='gamepad_node',
            parameters=[param_file_path],
	),
#	Node(
#	    package='racecar_mn',
#            node_executable='mux.py',
#            node_name='mux'
#	),
#	Node(
#	    package='racecar_mn',
#            node_executable='pwm.py',
#            node_name='pwm.py',
#	),
#	Node(
#	    package='racecar_mn',
#            node_executable='throttle.py',
#            node_name='throttle'
#	),
#        Node(
#            package='racecar_mn',
#            node_executable='simple_camera.py',
#            node_name='simple_camera'
#	)    
    ])


# # ros1 version
#<launch>
#  <!-- Launch the wall follower with parameters -->
#  <rosparam command="load" file="$(find racecar_mn)/config/params.yaml" />
#  <node pkg="joy" type="joy_node" name="joy_node" />
#  <node pkg="racecar_mn" name="mux" type="mux.py" output="screen" />
#  <node pkg="racecar_mn" name="gamepad" type="gamepad.py" output="screen" />
#  <node pkg="racecar_mn" name="pwm" type="pwm.py" output="screen" />
#  <node pkg="racecar_mn" name="throttle" type="throttle.py" output="screen" />
#
#  <!-- Uncomment to enable by default -->
#  <!-- <node pkg="racecar_mn" name="simple_camera" type="simple_camera.py" /> -->
#  <!-- <include file="$(find ydlidar)/launch/lidar.launch" /> -->
#</launch>


