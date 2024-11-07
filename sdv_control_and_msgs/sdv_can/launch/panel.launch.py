import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

   can_params = os.path.join(
      get_package_share_directory('sdv_can'),
      'config',
      'can_params.yaml'
   )

   dir_node = Node(
      package='sdv_can',
      executable='direcciondata.py',
      namespace="",
      output='screen',
      name='direcciondata',
      parameters=[can_params]
   )

   vel_node = Node(
      package='sdv_can',
      executable='velocitydata.py',
      namespace="",
      output='screen',
      name='velocitydata',
      parameters=[can_params]
   )
   panel_node = Node(
      package='sdv_can',
      executable='panel_modulev2.py',
      namespace="",
      output='screen',
      name='panel_modulev2',
      parameters=[can_params]
   )
   speech_node = Node(
    package='sdv_can',
    executable='speech.py',
    namespace="",
    output='screen',
    name='speech',
    parameters=[can_params]
    )
   return LaunchDescription([
      vel_node,
      dir_node,
      panel_node,
      speech_node
   ])
# sudo ifconfig enp7s0 10.66.171.22
