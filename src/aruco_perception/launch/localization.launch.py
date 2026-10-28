# Starts the whole localization system with a single command:
#   ros2 launch aruco_perception localization.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Parameters shared by several nodes, defined in one place
    reference_id = 1

    return LaunchDescription([
        # Camera driver
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            parameters=[{'pixel_format': 'mjpeg2rgb'}],
            output='screen',
        ),
        # Detects markers and estimates their 3D pose (camera frame)
        Node(
            package='aruco_perception',
            executable='detector_node',
            parameters=[{
                'reference_id': reference_id,
                'reference_marker_size': 0.195,  # metres
                'robot_marker_size': 0.10,       # metres
            }],
            output='screen',
        ),
        # Selects the floor reference marker (ground plane)
        Node(
            package='aruco_perception',
            executable='reference_node',
            parameters=[{'reference_id': reference_id}],
            output='screen',
        ),
        # Computes x, y, heading of every robot on the ground
        Node(
            package='aruco_perception',
            executable='robot_pos_node',
            parameters=[{'reference_id': reference_id}],
            output='screen',
        ),
    ])
