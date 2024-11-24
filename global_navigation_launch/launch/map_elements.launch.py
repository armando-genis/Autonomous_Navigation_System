import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    paramsConfig = os.path.join(get_package_share_directory('global_navigation_launch'),'config','map_params.yaml')

    map_visualizer_node = launch_ros.actions.Node(
        package='map_visualizer',
        executable='osm_visualizer',
        name='osm_visualizer',
        parameters=[paramsConfig],
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': "{message}"},
        output='screen',
    )

    publisher_node_planner = launch_ros.actions.Node(
        package='waypoints_routing',
        executable='waypoints_routing_node',
        name='waypoints_routing_node',
        output='screen',
        parameters=[paramsConfig],
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': "{message}"}
    )


    occupancy_map_node = launch_ros.actions.Node(
        package='map_visualizer',
        executable='occupancy_pub',
        name='occupancy_pub',
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': "{message}"},
        output='screen',
    )

    
    return launch.LaunchDescription([
        map_visualizer_node,
        publisher_node_planner,
        occupancy_map_node
    ])