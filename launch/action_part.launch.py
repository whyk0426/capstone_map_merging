from launch_ros.substitutions import FindPackageShare

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir

from launch_ros.actions import Node

import math

use_sim_time = LaunchConfiguration('use_sim_time', default='false')
resolution = LaunchConfiguration('resolution', default='0.05')
publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

pi = math.pi

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('capstone_map_merging'),
                    'launch',
                    'Lima_cartographer.launch.py'
                ])
            ),
        ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         PathJoinSubstitution([
        #             FindPackageShare('capstone_map_merging'),
        #             'launch',
        #             'Alpha_cartographer.launch.py',
        #         ])
        #     ),
        # ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('capstone_map_merging'),
                    'launch',
                    'Romeo_cartographer.launch.py',
                ])
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('capstone_map_merging'),
                    'launch',
                    'United_map.launch.py'
                ])
            ),
        ),
    
        # # Map Merging
        # Node(
        #     package='capstone_map_merging',
        #     executable='merge_map',
        #     output='screen',
        #     parameters=[{'use_sim_time': True}],
        #     remappings=[
        #         ("/map0", "/Lima/map"),
        #         #("/map1", "/Alpha/map"),
        #         ("/map1", "/Romeo/map"),
        #         ],
        # ),

        # repulsive_potential_itself
        Node(
            package='capstone_map_merging',
            executable='repulsive_force',
            remappings=[
                ("/map0", "/Lima/map"),
                ],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'Lima_map']
        ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_transform_publisher',
        #     output='screen',
        #     #arguments=['0', '0.5', '0', str(PI/4), '0', '0', 'map', 'Alpha_map'],
        #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'Alpha_map']
        # ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'Romeo_map']
        ),
    ])
