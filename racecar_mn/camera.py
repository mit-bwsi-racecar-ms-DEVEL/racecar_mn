#!/usr/bin/python3

# node abstraction for using maestro.py driver for servo/motor

# standard library imports
import sys
import time
import numpy as np

# ROS imports
import rclpy
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSProfile
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
# from geometry_msgs.msg import Quaternion
# from geometry_msgs.msg import Vector3

# cv stuff
from cv_bridge import CvBridge
import cv2 as cv

# camera import
import pyrealsense2 as rs

# for supported dimensions and rates refer to the
# Intel Realsense D400 Series Product Family datasheet in
# Section 4-2 Vision Processor D4 Data Streams
FRAME_RATE = 60
WIDTH = 640
HEIGHT = 480

# used for unit convertion g to m/s^2
# GRAVITATIONAL_ACCEL = 9.80665 

def main(args=None):
    # pipeline stuff
    pipeline = rs.pipeline()
    config = rs.config()
    
    # streams
    config.enable_stream(
            rs.stream.color, 
            WIDTH, 
            HEIGHT, 
            rs.format.bgr8, 
            FRAME_RATE
    )
    config.enable_stream(
            rs.stream.depth, 
            WIDTH, 
            HEIGHT, 
            rs.format.z16, 
            FRAME_RATE
    )
    # config.enable_stream(rs.stream.accel, format = rs.format.motion_xyz32f, framerate = 200 )
    # config.enable_stream(rs.stream.gyro, format = rs.format.motion_xyz32f, framerate = 200 )
    config.enable_stream(
            rs.stream.accel, 
            rs.format.motion_xyz32f, 
            250
    )
    config.enable_stream(
            rs.stream.gyro,
            rs.format.motion_xyz32f,
            200
    )

    # start pipeline
    profile = pipeline.start(config)

    # Get depth scale
    # depth_sensor = profile.get_device().first_depth_sensor()
    # depth_scale = depth_sensor.get_depth_scale()
    # align_to = rs.stream.color
    # align = rs.align(align_to)

    # init ros
    rclpy.init(args=args)
    node = rclpy.create_node('camera')

    try:
        qos_profile = QoSProfile(depth=1)
        qos_profile.history = QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST
        qos_profile.reliability = QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE
        
        # start publishers
        pub_gyro = node.create_publisher(Imu, '/camera/gyro', qos_profile)
        pub_accel = node.create_publisher(Imu, '/camera/accel', qos_profile)
        pub_camera_color = node.create_publisher(Image, '/camera/color', qos_profile)
        pub_camera_depth = node.create_publisher(Image, '/camera/depth', qos_profile)

        # create the cv bridge
        bridge = CvBridge()

        # initialize imu message
        msg_imu = Imu()
        msg_imu.orientation.w = 0.0
        msg_imu.orientation.x = 0.0
        msg_imu.orientation.y = 0.0
        msg_imu.orientation.z = 0.0
        msg_imu.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg_imu.angular_velocity_covariance = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg_imu.linear_acceleration_covariance = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        _image = {'Depth': None, 'Color': None}
        _encoding = {'Depth': '16UC1', 'Color': 'bgr8'}
        _publisher = {
            'Depth': pub_camera_depth,
            'Color': pub_camera_color
        }
        # _imu_frames = {'Accel': False, 'Gyro': True}
        # _imu_field = {'Gyro': 'angular_velocity' , 'Accel': 'linear_acceleration'} 
        
        # publish image frames to topics
        def publish_image(frame):
            stream_name = frame.get_profile().stream_name()
            height = 0
            width = 0
            bpp = 1
            _image[stream_name] = np.array(frame.get_data())


            if (frame.is_video_frame()):
                image = frame.as_video_frame()
                height = image.get_height()
                width = image.get_width()
                bpp = image.get_bytes_per_pixel()

            msg = bridge.cv2_to_imgmsg(_image[stream_name], _encoding[stream_name])
            msg.height = height
            msg.width = width 
            msg.is_bigendian = 0
            msg.step = bpp * width

            # publish message to ROS2 camera topics
            _publisher[stream_name].publish(msg)
        while rclpy.ok():
            try:
                # retrieve frames from the stream
                frames = pipeline.wait_for_frames()
                for frame in frames:
                    frame_type = frame.get_profile().stream_type()
                    if frame_type == rs.stream.color or frame_type == rs.stream.depth:
                        publish_image(frame)
                    # elif frame_type == rs.stream.accel or rs.stream.gyro:
                    #     stream_name = frame.get_profile().stream_name()
                    #     imu_data = frame.as_motion_frame().get_motion_data()
                    #     msg_imu.{_imu_field[stream_name]}.x = imu_data.x 
                    #     msg_imu.{_imu_field[stream_name]}.y = imu_data.y 
                    #     msg_imu.{_imu_field[stream_name]}.z = imu_data.z 
                    #     continue
                    elif frame_type == rs.stream.accel:
                        # _imu_frames['Accel'] = True
                        accel_data = frame.as_motion_frame().get_motion_data()
                        msg_imu.linear_acceleration.x = accel_data.x 
                        msg_imu.linear_acceleration.y = accel_data.y 
                        msg_imu.linear_acceleration.z = accel_data.z
                        pub_accel.publish(msg_imu)
                    elif frame_type == rs.stream.gyro:
                        # _imu_frames['Gyro'] = True
                        gyro_data = frame.as_motion_frame().get_motion_data()
                        msg_imu.angular_velocity.x = gyro_data.x
                        msg_imu.angular_velocity.y = gyro_data.y
                        msg_imu.angular_velocity.z = gyro_data.z
                        pub_gyro.publish(msg_imu)
                    else:
                        print("Found alternative frame type: ", frame_type)
                
                # if _imu_frames['Accel'] and _imu_frames['Gyro']:
                #     pub_imu.publish(msg_imu)
                # _imu_frames['Accel'] = False
                # _imu_frames['Gyro'] = False
                
                
                # Align the depth frame to color frame (frames must be the depth and color)
                # aligned_frames = align.process(frames)

                # Get aligned frames
                # aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
                # color_frame = aligned_frames.get_color_frame()


                # if not aligned_depth_frame or not color_frame:
                #     continue
                # depth_image = np.asanyarray(aligned_depth_frame.get_data())
                # color_image = np.asanyarray(color_frame.get_data())

                
                # cv.namedWindow('Image window', cv.WINDOW_AUTOSIZE)
                # cv.imshow("Image window", color_image)
                # cv.waitKey(1)
            except RuntimeError as e:
                print(e)

    except KeyboardInterrupt:
        pass
    finally:
        pipeline.stop()
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)