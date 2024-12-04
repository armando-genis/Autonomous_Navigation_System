import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

   car_guidance_node = Node(
      package='sdv_control',
      executable='stanley_controller_node',
      output='screen',
      emulate_tty=True,
      arguments=[('__log__level:=debug')],
      name='stanley_controller_node',
      parameters=[
                  car_params
                  ]
   )

   pid_node = Node(
      package='sdv_control',
      executable='vel_pid_node',
      parameters=[car_params],
   )

   pid_regulator_node = Node(
      package='sdv_control',
      executable='velocity_regulator_node',
   )

   return LaunchDescription([
      car_guidance_node,
      pid_node,
      pid_regulator_node,
   ])