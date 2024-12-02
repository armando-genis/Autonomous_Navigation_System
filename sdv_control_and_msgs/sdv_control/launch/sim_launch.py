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

   # For simulations (config in sdv_control)
   rviz_config = os.path.join(
      get_package_share_directory('sdv_control'),
      'rviz',
      'sim.rviz'
   )

   rviz = Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2',
      arguments=['-d', rviz_config],
   )

   sim_params = os.path.join(
      get_package_share_directory('sdv_control'),
      'config',
      'sim_params.yaml'
   )

   pid_node = Node(
      package='sdv_control',
      executable='vel_pid_node',
      parameters=[sim_params],
   )

   pid_regulator_node = Node(
      package='sdv_control',
      executable='velocity_regulator_node',
   )

   stanley_node = Node(
      package='sdv_control',
      executable='stanley_controller_node',
      parameters=[sim_params],
   )

   path_planner_node = Node(
      package='sdv_control',
      executable='path_planner_node.py',
   )

   path_publisher_node = Node(
      package='sdv_control',
      executable='path_publisher_node.py',
   )

   kinematic_node = Node(
      package='sdv_control',
      executable='sdv_kinematic_sim',
      remappings=[
         ("/input/accel_x", "/sdv/velocity/throttle"),
         ("/output/odom", "/vectornav/velocity_body"),
         ("/input/steering", "/sdv/steering/delta_setpoint"),
      ]
   )

   foxglove_bridge = Node(
      name="foxglove_bridge",
      package="foxglove_bridge",
      executable="foxglove_bridge")

   return LaunchDescription([
      rviz,
      pid_node,
      pid_regulator_node,
      kinematic_node,   
      stanley_node,
      path_planner_node,
      path_publisher_node,

      foxglove_bridge,
      
      # waypoint_handler,
   ])