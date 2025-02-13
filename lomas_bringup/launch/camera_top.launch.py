#!/usr/bin/env python3
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression

def generate_launch_description():

    # Lomas camera node
    lomas_top_camera_node = Node(
        package = 'camera_ros',
        executable = 'camera_node',
        name = 'lomas_top_camera_node',
        parameters = [{
            'width': 800,
            'height': 600
        }],
        remappings = [
            ('/lomas_top_camera_node/camera_info', '/lomas/top/camera/info'),
            ('/lomas_top_camera_node/image_raw', '/lomas/top/camera/image'),
            ('/lomas_top_camera_node/image_raw/compressed', '/lomas/top/camera/image/compressed')
        ]
    )

    return LaunchDescription([
        lomas_top_camera_node
    ])
