import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    map_visualizer_node = launch_ros.actions.Node(
        package='map_visualizer',
        executable='osm_visualizer',
        name='osm_visualizer',
        parameters=[
            {"map_path": "/home/genis/Desktop/sdv_ws/src/Autonomous_Navigation_System/map_visualizer/osms/mty_map_full_2.osm"}
        ],
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': "{message}"},
        output='screen',
    )

    occupancy_map_node = launch_ros.actions.Node(
        package='map_visualizer',
        executable='occupancy_pub',
        name='occupancy_pub',
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': "{message}"},
        output='screen',
    )

    paramsConfig = os.path.join(get_package_share_directory('waypoints_routing'),'config','params.yaml')

    publisher_node_planner = launch_ros.actions.Node(
        package='waypoints_routing',
        executable='waypoints_routing_node',
        name='waypoints_routing_node',
        output='screen',
        parameters=[paramsConfig],
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': "{message}"}
    )

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
        map_visualizer_node,
        occupancy_map_node, 
        publisher_node_planner
        # waypoints_index_node
    ])