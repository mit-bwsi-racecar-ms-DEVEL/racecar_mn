#!/usr/bin/python3
from ament_index_python.packages import get_package_share_directory

import launch
import launch.actions
import launch.substitutions
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
#    share_dir = get_package_share_directory('ydlidar')
#    ydlidar_launch = IncludeLaunchDescription(
#            PythonLaunchDescriptionSource([
#                share_dir, '/launch/ydlidar_launch.py']))
    
    # param_file_path = '/home/racecar/racecar_ws/base_ws/src/racecar_mn/config/params.yaml'
    
    return launch.LaunchDescription([
	Node(
	    package='joy',
            node_executable='joy_node',
            node_name='joy_node',
	),
	Node(
	    package='racecar_mn',
            node_executable='pwm',
            node_name='pwm_node',
	),
        Node(
            package='racecar_mn',
            node_executable='camera',
            node_name='camera_node',
        )
#        ),
#        ydlidar_launch
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


