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
  lidar_pkg_share = get_package_share_directory('sllidar_ros2')

  # Launch parameters
  lidar_serial_port = LaunchConfiguration('lidar_serial_port')

  # Base rplidar node
  rplidar_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      os.path.join(
        lidar_pkg_share, 'launch', 'sllidar_a1_launch.py'
      )
    ]),
    launch_arguments={
      'channel_type'    : 'serial',
      'serial_port'     : lidar_serial_port,
      'serial_baudrate' : '115200',
      'frame_id'        : 'laser_frame',
      'inverted'        : 'false',
      'angle_compensate': 'true',
      'scan_mode'       : 'Sensitivity'
    }.items()
  )
  
  return LaunchDescription([
    # Launch Parameters
    DeclareLaunchArgument(
      'lidar_serial_port',
      default_value='/dev/rplidar',
      description='Serial port that Lidar is connected to'),
    
    rplidar_node
  ])