import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    paramsConfig = os.path.join(get_package_share_directory('target_waypoint_index'),'config','params.yaml')


    waypoints_index_node = launch_ros.actions.Node(
        package='target_waypoint_index',
        executable='target_waypoint_index_node',
        name='target_waypoint_index_node',
        parameters=[paramsConfig],
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': "{message}"},
        output='screen'
    )
    
    return launch.LaunchDescription([
        waypoints_index_node
    ])
