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
  
  # Lidar
  lidar = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      os.path.join(
        launch_pkg_share, 'launch', 'lidar.launch.py'
      )
    ])
  )
  
  # Filtered Laser
  filtered_laser_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      os.path.join(
        launch_pkg_share, 'launch', 'filtered_laser.launch.py'
      )
    ])
  )
  
  # Delay the Lidar
  delayed_lidar = TimerAction(
    period = 2.5,
    actions = [lidar]
  )
  
  # Delay the Filtered Lidar
  delayed_filter_laser = TimerAction(
    period = 5.0,
    actions = [filtered_laser_node]
  )
  
  return LaunchDescription([
    delayed_lidar,
    delayed_filter_laser
  ])