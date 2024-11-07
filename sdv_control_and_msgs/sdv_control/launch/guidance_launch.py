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

   is_sim = DeclareLaunchArgument(  
      'is_simulation',
      default_value = 'false',
      description = 'Defines if the application will run in simulation or in real life'
   )

   car_params = os.path.join(
      get_package_share_directory('sdv_control'),
      'config',
      'car_params.yaml'
   )

   car_guidance_node = Node(
      package='sdv_control',
      executable='stanley_controller_node',
      output='screen',
      name='stanley_controller_node',
      parameters=[
                  {'is_simulation': LaunchConfiguration('is_simulation')},
                  car_params
                  ]
   )

   waypoint_handler = Node(
      package='sdv_control',
      executable='waypoint_handler.py',
      namespace="",
      output="screen",
      name='waypoint_handler',
      parameters=[car_params]
   )

   return LaunchDescription([
      is_sim,

      LogInfo(
         condition=IfCondition(LaunchConfiguration('is_simulation')),
         msg="Running in simulation mode."
      ),
      LogInfo(
         condition=UnlessCondition(LaunchConfiguration('is_simulation')),
         msg="Running in real robot mode."
      ),

      waypoint_handler,
      car_guidance_node,
   ])