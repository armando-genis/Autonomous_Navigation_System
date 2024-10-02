import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():

    publisher_node = launch_ros.actions.Node(
        package='lidar_path_to_odom',
        executable='lidar_path_to_odom_node',
        name='lidar_path_to_odom',
        output='screen',

    )
    
    return launch.LaunchDescription([
        publisher_node
    ])