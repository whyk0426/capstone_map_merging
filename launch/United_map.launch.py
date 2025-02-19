import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration



def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    package_dir = get_package_share_directory('capstone_turtlebot_controller')
    rviz_config_dir = os.path.join(package_dir, 'config', 'united_map.rviz')

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_transform_publisher',
        #     output='screen',
        #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'Merge_map']
        # ),
    ])