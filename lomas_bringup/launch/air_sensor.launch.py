#!/usr/bin/env python3
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression

def generate_launch_description():

    # Lomas air sensor node
    air_sensor_node = Node(
        package = 'lomas_sensors',
        executable = 'bme680',
        name = 'air_sensor_node',
        remappings = [
            ('/air/temperature', '/lomas/air/temperature'),
            ('/air/humidity', '/lomas/air/humidity'),
            ('/air/pressure', '/lomas/air/pressure')
        ]
    )
    
    return LaunchDescription([
        air_sensor_node
    ])
