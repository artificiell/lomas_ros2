#!/usr/bin/env python3
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression

def generate_launch_description():

    # Lomas camera node
    lomas_camera_node = Node(
        package = 'camera_ros',
        executable = 'camera_node',
        name = 'lomas_camera_node',
        remappings = [
            ('/lomas_camera_node/camera_info', '/lomas/camera/info'),
            ('/lomas_camera_node/image_raw', '/lomas/camera/image'),
            ('/lomas_camera_node/image_raw/compressed', '/lomas/camera/image/compressed')
        ]
    )

    return LaunchDescription([
        lomas_camera_node
    ])
