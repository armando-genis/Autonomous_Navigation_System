import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    paramsConfig = os.path.join(get_package_share_directory('global_navigation_launch'),'config','rotation_params.yaml')

    publisher_node_rotation = launch_ros.actions.Node(
        package='pointcloud_rotation',
        executable='pointcloud_rotation_node',
        name='pointcloud_rotation_node',
        output='screen',
        parameters=[paramsConfig],
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': "{message}"}
    )

    publisher_node_lidar_imu_sync = launch_ros.actions.Node(
        package='lidar_imu_sync',
        executable='lidar_imu_fusion_node',
        name='lidar_imu_fusion_node',
        output='screen',
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': "{message}"}
    )
    
    return launch.LaunchDescription([
        publisher_node_rotation,
        publisher_node_lidar_imu_sync
    ])