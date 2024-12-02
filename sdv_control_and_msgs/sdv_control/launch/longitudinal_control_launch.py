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

   car_params = os.path.join(
      get_package_share_directory('sdv_control'),
      'config',
      'car_params.yaml'
   )

   pid_node = Node(
      package='sdv_control',
      executable='vel_pid_node',
      parameters=[car_params],
   )

   foxglove_bridge = Node(
      name="foxglove_bridge",
      package="foxglove_bridge",
      executable="foxglove_bridge")
   
   can_node = Node(
      package='sdv_can',
      executable='sdv_can_node',
   )

   vectornav_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
            PathJoinSubstitution([
               FindPackageShare('vectornav'),
               'launch',
               'vectornav.launch.py'
            ])
      ])
   )

   return LaunchDescription([
      pid_node,
      # foxglove_bridge,
      # can_node,
      # vectornav_launch,
   ])