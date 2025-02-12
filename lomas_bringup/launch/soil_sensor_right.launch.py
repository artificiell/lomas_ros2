#!/usr/bin/env python3
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression

def generate_launch_description():

    # Launch configuration
    sensor_channel = LaunchConfiguration('sensor_channel')

    # Launch arguments
    sensor_channel_arg = DeclareLaunchArgument(
        'sensor_channel',
        default_value = '0'
    )
 
    # Lomas soil sensor node
    soil_sensor_node = Node(
        package = 'lomas_sensors',
        executable = 'mcp3002',
        name = 'soil_sensor_right_node',
        parameters = [{
            'channel': LaunchConfiguration('sensor_channel')
        }],
        remappings = [
            ('/soil/moisture', '/lomas/soil/moisture/right')
        ]
    )

    return LaunchDescription([
        sensor_channel_arg,
        soil_sensor_node
    ])
