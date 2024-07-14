from launch import LaunchDescription
from launch_ros.actions import Node
# from launch.substitutions import LaunchConfiguration
from math import pi
# from rosbag2.launch_actions import Record
# import os
# from ament_index_python.packages import get_package_share_directory
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource

# Ref
# https://github.com/foxglove/ros-foxglove-bridge

world2cam = "--x {x} --y {y} --z {z} ".format(x=0.2, y=0.0, z=0.2)
world2cam += "--yaw {yaw} --pitch {pitch} --roll {roll} ".format(yaw=0*pi/180, pitch=0*pi/180, roll=0*pi/180)
world2cam += "--child-frame-id {child} --frame-id {parent}".format(child="camera", parent="world")
world2cam = world2cam.split(' ')

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            parameters=[
                {"port": 8765},
                {"address": "0.0.0.0"},
                {"tls": False},
                {"topic_whitelist": [".*"]},
                {"send_buffer_limit": 10000000},
                {"use_sim_time": False},
                {"num_threads": 0}
            ]
        ),
        Node(
            package='rtf_pyopencv_camera',
            executable='pycamera_node',
            name='camera'
        ),
        Node(
            package = "tf2_ros",
            executable = "static_transform_publisher",
            arguments = world2cam
        ),
        # Record(
        #     name='my_recorder',
        #     parameters=[{
        #         'topics': LaunchConfiguration('my_topic_list')
        #     }]
        # ),
    ])
