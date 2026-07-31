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
        respawn = True,
        respawn_delay = 3.0,
    )

    # Lomas machine parameter store node
    lomas_store_node = Node(
        package = 'lomas_bridge',
        executable = 'store',
        name = 'machine_position_store_node'
    )

    # Lomas light plug node
    lomas_light_node = Node(
        package = 'lomas_bridge',
        executable = 'light',
        name = 'light_plug_node',
        parameters = [{
            'username': 'andreas.persson@oru.se',
            'password': 'Lomas2025',
            'ip_address':  '192.168.0.110'
        }],
    )
    
    return LaunchDescription([
        motor_speed_arg,
        lomas_bridge_node,
        lomas_store_node,
        lomas_light_node
    ])
