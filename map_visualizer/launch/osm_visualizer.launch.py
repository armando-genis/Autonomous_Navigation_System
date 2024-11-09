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
    
    return launch.LaunchDescription([
        map_visualizer_node
    ])
