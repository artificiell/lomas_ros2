#!/usr/bin/env python3
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression

def generate_launch_description():

    # Launch configuration
    motor_speed = LaunchConfiguration('motor_speed')

    # Launch arguments
    motor_speed_arg = DeclareLaunchArgument(
        'motor_speed',
        default_value = '100'
    )

    # Lomas machine bridge node
    lomas_bridge_node = Node(
        package = 'lomas_bridge',
        executable = 'machine',
        name = 'machine_bridge_node',
        parameters = [{
            'speed': LaunchConfiguration('motor_speed')
        }],
        remappings = [
            ('/machine/cmd', '/lomas/machine/cmd'),
            ('/machine/status', '/lomas/machine/status'),
        ]
    )

    # Lomas command converter node
    lomas_converter_node = Node(
        package = 'lomas_bridge',
        executable = 'converter',
        name = 'command_converter_node',
        remappings = [
            ('/converter/cmd', '/lomas/converter/cmd'),
            ('/machine/cmd', '/lomas/machine/cmd')
        ]
    )

    return LaunchDescription([
        motor_speed_arg,
        lomas_bridge_node,
        lomas_converter_node
    ])
