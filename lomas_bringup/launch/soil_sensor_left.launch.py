#!/usr/bin/env python3
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression

def generate_launch_description():

    # Launch configuration
    auto_calibration = LaunchConfiguration('auto_calibration')
    sensor_channel = LaunchConfiguration('sensor_channel')

    # Launch arguments
    auto_calibration_arg = DeclareLaunchArgument(
        'auto_calibration',
        default_value = 'true'
    )
    sensor_channel_arg = DeclareLaunchArgument(
        'sensor_channel',
        default_value = '0'
    )
 
    # Lomas soil sensor node
    soil_sensor_node = Node(
        package = 'lomas_sensors',
        executable = 'mcp3002',
        name = 'soil_sensor_left_node',
        parameters = [{
            'calibration': LaunchConfiguration('auto_calibration'),
            'channel': LaunchConfiguration('sensor_channel')
        }],
        remappings = [
            ('/soil/moisture', '/lomas/soil/moisture/left')
        ]
    )

    return LaunchDescription([
        auto_calibration_arg,
        sensor_channel_arg,
        soil_sensor_node
    ])
