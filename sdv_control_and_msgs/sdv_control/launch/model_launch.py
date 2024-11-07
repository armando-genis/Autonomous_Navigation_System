import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
   frequency_arg = DeclareLaunchArgument(
      name='frequency',
      default_value='100',
      description='Frequency for nodes'
   )

   rviz_config = os.path.join(
      get_package_share_directory('sdv_control'),
      'launch/rviz_cfg',
      'sdv.rviz'
   )

   car_model_node = Node(
      package='sdv_control',
      executable='sdc1_simulation_node',
      output='screen',
      name='sdc1_simulation_node',
      parameters=[{'frequency': LaunchConfiguration('frequency')}]
   )

   tf2_node = Node(
      package='sdv_control',
      executable='car_tf2_broadcast_node',
      namespace="",
      name='car_tf2_broadcast_node',
      parameters=[{'frequency': LaunchConfiguration('frequency')}]
   )

   rviz = Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2',
      # arguments=['-d', rviz_config]
   )

   return LaunchDescription([
      frequency_arg,
      car_model_node,
      # tf2_node,
      # rviz
   ])


   # return LaunchDescription([
      # nodes_frequency,

      # launch_ros.actions.Node(
      #    package='sdv_manual_control',
      #    executable='sdv_manual_control',
      #    name=['sdv_manual_control', launch.substitutions.LaunchConfiguration('role_name')],
      #    output='screen',
      #    emulate_tty=True,
      #    parameters=[
      #          {
      #             'role_name': launch.substitutions.LaunchConfiguration('role_name')
      #          }
      #    ]
      # ),

   # ])

    