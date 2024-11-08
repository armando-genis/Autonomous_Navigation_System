import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

    # ==========================> CONTROL NODES <==========================

    is_sim = DeclareLaunchArgument(
        'is_simulation',
        default_value = 'false',
        description = 'Defines if the application will run in simulation or in real life'
    )

    odometry_source = DeclareLaunchArgument(
        'odometry_source',
        default_value = 'vn',
        description = 'Defines if the odometry source comes directly from the \
                        vectornav (vn) or from the robot localization pkg (rl)'
    )


    car_params = os.path.join(
        get_package_share_directory('sdv_control'),
        'config',
        'car_params.yaml'
    )

    pid_node = launch_ros.actions.Node(
        package='sdv_control',
        executable='sdc1_vel_pid_node',
        output='screen',
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': "{message}"},
        name='sdc1_vel_pid_node',
        parameters=[
                    # {'frequency': LaunchConfiguration('frequency')},
                    {'is_simulation': LaunchConfiguration('is_simulation')},
                    # {'odometry_source': LaunchConfiguration('odometry_source')},
                    car_params
                    ]
    )

    car_guidance_node = launch_ros.actions.Node(
        package='sdv_control',
        executable='stanley_controller_node',
        output='screen',
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': "{message}"},
        name='stanley_controller_node',
        parameters=[
                    # {'frequency': LaunchConfiguration('frequency')},
                    {'is_simulation': LaunchConfiguration('is_simulation')},
                    car_params
                    ]
    )

    waypoint_handler = launch_ros.actions.Node(
        package='sdv_control',
        executable='waypoint_handler.py',
        namespace="",
        output="screen",
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': "{message}"},
        name='waypoint_handler',
        parameters=[car_params]
    )

    # ==========================> CAN NODE <==========================

    sdv_control_launch = launch_ros.actions.Node(
        package='sdv_can',
        executable='sdv_can_node',
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': "{message}"},
        output='screen',
    )

    sdv_joy = launch_ros.actions.Node(
        package='joy',
        executable='joy_node',
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': "{message}"},
        output='screen',
    )



    # ==========================> CAN DIVICES <==========================
    
    return launch.LaunchDescription([

        # CAN nodes
        sdv_control_launch,
        sdv_joy,

        # Control nodes
        is_sim,
        odometry_source, 
        # Log the value of is_simulation for debugging purposes
        LogInfo(
            condition=IfCondition(LaunchConfiguration('is_simulation')),
            msg="Running in simulation mode."
        ),
        LogInfo(
            condition=UnlessCondition(LaunchConfiguration('is_simulation')),
            msg="Running in real robot mode."
        ),
        pid_node,      
        waypoint_handler,
        car_guidance_node,
    ])