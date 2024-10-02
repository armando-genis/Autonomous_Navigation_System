import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():

    publisher_node = launch_ros.actions.Node(
        package='lidar_imu_sync',
        executable='lidar_imu_fusion_node',
        name='lidar_imu_fusion_node',
        output='screen',

    )
    
    return launch.LaunchDescription([
        publisher_node
    ])