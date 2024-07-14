from launch import LaunchDescription
from launch_ros.actions import Node
# from launch.substitutions import LaunchConfiguration

# from rosbag2.launch_actions import Record
# import os
# from ament_index_python.packages import get_package_share_directory
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource

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
        # Record(
        #     name='my_recorder',
        #     parameters=[{
        #         'topics': LaunchConfiguration('my_topic_list')
        #     }]
        # ),
    ])
