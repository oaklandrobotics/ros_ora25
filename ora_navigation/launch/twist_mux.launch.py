import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
  pkg_share = get_package_share_directory('ora_navigation')

  twist_mux_params = os.path.join(pkg_share, 'config', 'twist_mux.yaml')
  
  use_sim_time = LaunchConfiguration('use_sim_time')

  twist_mux_node = Node(
    package='twist_mux',
    executable='twist_mux',
    parameters=[twist_mux_params, {use_sim_time: use_sim_time}],
    remappings=[
      ('/cmd_vel_out', '/diff_drive_cont/cmd_vel_unstamped'),
    ],
  )

  return LaunchDescription([
    DeclareLaunchArgument(
      'use_sim_time',
      default_value='true',
      description='Use sim time if true'),
    
    twist_mux_node,
  ])

