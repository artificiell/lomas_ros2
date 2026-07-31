#!/usr/bin/env python3
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Launch configuration
    motor_speed = LaunchConfiguration('motor_speed')
    service_name = LaunchConfiguration('service_name')
    
    # Launch arguments
    motor_speed_arg = DeclareLaunchArgument(
        'motor_speed',
        default_value = '100'
    )
    service_name_arg = DeclareLaunchArgument(
        'service_name',
        default_value='/lomas/top/camera/image'
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
        ],
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
    
    # Lomas demo node
    lomas_demo_node = Node(
        package = 'lomas_bridge',
        executable = 'demo',
        name = 'lomas_demo_node',
        remappings = [
            ('/machine/start', '/lomas/machine/start'),
            ('/machine/stop', '/lomas/machine/stop')
        ]
    )

    # Include top camera launch
    top_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('lomas_bringup'), 'launch', 'camera_top.launch.py'])
        ),
        launch_arguments = {
            'service_name': service_name,
        }.items()
    )
    

    return LaunchDescription([
        motor_speed_arg,
        service_name_arg,
        lomas_bridge_node,
        lomas_store_node,
        lomas_light_node,
        lomas_demo_node,
        top_camera_launch
    ])
