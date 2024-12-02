from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sdv_can',
            executable='remote2can.py',
        ),
        Node(
            package='sdv_can',
            executable='sdv_can_node',
            remappings=[
            ("/sdv/throttle/setpoint", "/sdc_control/control_signal/D"),
            ]
        ),
        Node(
            package='joy',
            executable='joy_node',
        )
    ])