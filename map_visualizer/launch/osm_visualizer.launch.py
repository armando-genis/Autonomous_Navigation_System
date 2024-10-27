from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="map_visualizer",
            executable="osm_visualizer",
            name="osm_visualizer",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"map_path": "/home/genis/Desktop/sdv_ws/src/Autonomous_Navigation_System/map_visualizer/osms/mty_map_full_5.osm"}
            ]
        )
    ])