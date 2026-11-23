# Starts the whole localization system with a single command:
#   ros2 launch aruco_perception localization.launch.py
# All parameters are read from config/params.yaml.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('aruco_perception'), 'config', 'params.yaml')

    return LaunchDescription([
        Node(package='usb_cam', executable='usb_cam_node_exe',
             parameters=[params], output='screen'),
        Node(package='aruco_perception', executable='detector_node',
             parameters=[params], output='screen'),
        Node(package='aruco_perception', executable='reference_node',
             parameters=[params], output='screen'),
        Node(package='aruco_perception', executable='robot_pos_node',
             parameters=[params], output='screen'),
    ])
