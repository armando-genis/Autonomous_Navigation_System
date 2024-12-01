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

   odometry_source = DeclareLaunchArgument(
      'odometry_source',
      default_value = 'vn',
      description = 'Defines if the odometry source comes directly from the \
                     vectornav (vn) or from the robot localization pkg (rl)'
   )

   # For simulations (config in sdv_control)
   rviz_config = os.path.join(
      get_package_share_directory('sdv_control'),
      'launch/rviz_cfg',

      'sdv_sim_wpnts.rviz'
   )

   car_params = os.path.join(
      get_package_share_directory('sdv_control'),
      'config',
      'car_params.yaml'
   )

   pid_node = Node(
      package='sdv_control',
      executable='sdc1_vel_pid_node',
      output='screen',
      name='sdc1_vel_pid_node',
      parameters=[
                  # {'frequency': LaunchConfiguration('frequency')},
                  {'is_simulation': LaunchConfiguration('is_simulation')},
                  # {'odometry_source': LaunchConfiguration('odometry_source')},
                  car_params
                  ]
   )

   asmc_node = Node(
      package='sdv_control',
      executable='sdc1_vel_asmc_node',
      output='screen',
      name='sdc1_vel_asmc_node',
      parameters=[
                  # {'frequency': LaunchConfiguration('frequency')},
                  {'is_simulation': LaunchConfiguration('is_simulation')},
                  car_params
                  ]
   )

   aitsmc_node = Node(
      package='sdv_control',
      executable='sdc1_vel_aitsmc_node',
      output='screen',
      name='sdc1_vel_aitsmc_node',
      parameters=[
                  # {'frequency': LaunchConfiguration('frequency')},
                  {'is_simulation': LaunchConfiguration('is_simulation')},
                  car_params
                  ]
   )

   car_guidance_node = Node(
      package='sdv_control',
      executable='stanley_controller_node',
      output='screen',
      name='stanley_controller_node',
      parameters=[
                  # {'frequency': LaunchConfiguration('frequency')},
                  {'is_simulation': LaunchConfiguration('is_simulation')},
                  car_params
                  ]
   )

   tf2_node = Node(
      package='sdv_control',
      executable='car_tf2_broadcast_node',
      namespace="",
      name='car_tf2_broadcast_node',
      parameters=[
                  # {'frequency': LaunchConfiguration('frequency')},
                  car_params
                  ],
      condition=IfCondition(LaunchConfiguration('is_simulation'))
   )

   rviz = Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2',
      arguments=['-d', rviz_config],
      # condition=IfCondition(LaunchConfiguration('is_simulation'))
   )

   waypoint_handler = Node(
      package='sdv_control',
      executable='waypoint_handler.py',
      namespace="",
      output="screen",
      name='waypoint_handler',
      parameters=[car_params]
   )

   sdv_loc_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
            PathJoinSubstitution([
               FindPackageShare('sdv_localization'),
               'launch',
               'sdv_localization.launch.py'
            ])
      ]),
      launch_arguments={'is_simulation': LaunchConfiguration('is_simulation'),
                        'odometry_source': LaunchConfiguration('odometry_source')}.items()
   )

   sdv_description_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
            PathJoinSubstitution([
               FindPackageShare('sdv_description'),
               'launch',
               'rviz.launch.py'
            ])
      ])
   )

   sdv_can_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
            PathJoinSubstitution([
               FindPackageShare('sdv_can'),
               'launch',
               'can_devices.launch.py'
            ])
      ]),
   )

   return LaunchDescription([
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

      # rviz,
      # aitsmc_node,
      # asmc_node,
      pid_node,
      # tf2_node,
      # sdv_description_launch,
      # sdv_loc_launch,
      
      # sdv_can_launch,
      
      waypoint_handler,
      car_guidance_node,
   ])