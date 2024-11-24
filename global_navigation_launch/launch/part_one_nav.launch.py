
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

# ANSI escape codes for colored terminal output
green = "\033[1;32m"
red = "\033[1;31m"
blue = "\033[1;34m"
yellow = "\033[1;33m"
purple = "\033[1;35m"
reset = "\033[0m"

def generate_launch_description():

    # ros2 launch sensors_launch lidar_imu.launch.py
    Sensor_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sensors_launch'),
                'launch',
                'lidar_imu.launch.py'
            ])
        ]),
    )

    # ros2 launch global_navigation_launch lidar_subprocessing.launch.py
    sensor_processing = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('global_navigation_launch'),
                'launch',
                'lidar_subprocessing.launch.py'
            ])
        ]),
    )

    # ros2 launch robot_description localization_display.launch.py
    robot_description_for_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_description'),
                'launch',
                'localization_display.launch.py'
            ])
        ]),
    )

    # ros2 launch global_navigation_launch lidar_localization_ros2.launch.py
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('global_navigation_launch'),
                'launch',
                'lidar_localization_ros2.launch.py'
            ])
        ]),
    )

    # ros2 launch global_navigation_launch map_elements.launch.py
    map_elements = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('global_navigation_launch'),
                'launch',
                'map_elements.launch.py'
            ])
        ]),
    )

    return LaunchDescription([
        Sensor_launcher,
        sensor_processing,
        robot_description_for_localization,
        map_elements,
        
        TimerAction(
            actions=[
                localization
            ],
            period='3.0',  
        ),
        
    ])