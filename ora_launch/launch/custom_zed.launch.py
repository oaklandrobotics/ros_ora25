import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, RegisterEventHandler
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.event_handlers import OnProcessStart

def generate_launch_description():
  launch_pkg_share = get_package_share_directory('ora_launch')

  custom_zed_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      os.path.join(
        launch_pkg_share, 'launch', 'base_zed.launch.py'
      )
    ]),
    launch_arguments={
       'camera_model': 'zed2i',
       'sim_mode': 'false',
       'use_sim_time': 'false',
       'publish_tf': 'false',
       'publish_map_tf': 'false'
     }.items()
  )
  
  return LaunchDescription([
    custom_zed_node
  ])