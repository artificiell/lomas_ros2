#!/usr/bin/env python3
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression

def generate_launch_description():

    # Lomas camera node
    lomas_side_camera_node = Node(
        package = 'camera_ros',
        executable = 'camera_node',
        name = 'lomas_side_camera_node',
        remappings = [
            ('/lomas_side_camera_node/camera_info', '/lomas/side/camera/info'),
            ('/lomas_side_camera_node/image_raw', '/lomas/side/camera/image'),
            ('/lomas_side_camera_node/image_raw/compressed', '/lomas/side/camera/image/compressed')
        ]
    )

    return LaunchDescription([
        lomas_side_camera_node
    ])
