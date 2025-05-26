import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
  pkg_share = get_package_share_directory('ora_navigation')
  teleop_pkg_share = get_package_share_directory('ora_teleop')
  slam_toolbox_pkg_share = get_package_share_directory('ora_slam_toolbox')
  
  use_sim_time = LaunchConfiguration('use_sim_time')

  twist_mux = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      os.path.join(
        pkg_share,'launch','twist_mux.launch.py'
      )
    ]),
    launch_arguments={'use_sim_time': 'false'}.items()
  )
  
  teleop = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      os.path.join(
        teleop_pkg_share,'launch','joystick.launch.py'
      )
    ])
  )
  
  slam_toolbox = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      os.path.join(
        slam_toolbox_pkg_share,'launch','slam_toolbox.launch.py'
      )
    ]),
    launch_arguments={'use_sim_time': 'false'}.items()
  )
  
  robot_localization_node = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_node',
    output='screen',
    parameters=[
      os.path.join(pkg_share, 'config/real_ekf.yaml'),
      {'use_sim_time': LaunchConfiguration('use_sim_time')}
    ]
  )

  return LaunchDescription([
    DeclareLaunchArgument(
      'use_sim_time',
      default_value='false',
      description='Use sim time if true'),
    
    twist_mux,
    teleop,
    slam_toolbox,
    # robot_localization_node
  ])

