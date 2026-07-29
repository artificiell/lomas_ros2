#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Launch configuration
    sharpness = LaunchConfiguration('sharpness')
    denoise = LaunchConfiguration('denoise')
    shutter = LaunchConfiguration('shutter')
    gain = LaunchConfiguration('gain')
    ev = LaunchConfiguration('ev')
    awb = LaunchConfiguration('awb')
    brightness = LaunchConfiguration('brightness')
    contrast = LaunchConfiguration('contrast')
    service_name = LaunchConfiguration('service_name')

    # Launch arguments
    sharpness_arg = DeclareLaunchArgument(
        'sharpness',
        default_value='1.5'
    )
    denoise_arg = DeclareLaunchArgument(
        'denoise',
        default_value='cdn_hq',
        description='off / cdn_off, cdn_fast, cdn_hq, auto'
    )
    shutter_arg = DeclareLaunchArgument(
        'shutter',
        default_value='160000',
        description='ExposureTime in microseconds, 0 = auto.'
    )
    gain_arg = DeclareLaunchArgument(
        'gain',
        default_value='16.0',
        description='AnalogueGain, 0.0 = auto.'
    )
    ev_arg = DeclareLaunchArgument(
        'ev',
        default_value='2.0',
        description='ExposureValue in stops, 0.0 = none.'
    )
    awb_arg = DeclareLaunchArgument(
        'awb',
        default_value='auto',
        description='auto, indoor, tungsten, fluorescent, daylight'
    )
    brightness_arg = DeclareLaunchArgument(
        'brightness',
        default_value='0.0'
    )
    contrast_arg = DeclareLaunchArgument(
        'contrast',
        default_value='1.0'
    ) 
    service_name_arg = DeclareLaunchArgument(
        'service_name',
        default_value='/lomas/top/camera/image'
    )
    
    # Include global shutter camera lanch
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('rpi_gs_cam'), 'launch', 'camera.launch.py'])
        ),
        launch_arguments={
            'sharpness': sharpness,
            'denoise': denoise,
            'shutter': shutter,
            'gain': gain,
            'ev': ev,
            'awb': awb,
            'brightness': brightness,
            'contrast': contrast,
            'service_name': service_name,
        }.items()
    )
  
    return LaunchDescription([
        sharpness_arg,
        denoise_arg,
        shutter_arg,
        gain_arg,
        ev_arg,
        awb_arg,
        brightness_arg,
        contrast_arg,
        service_name_arg,
        camera_launch        
    ])
