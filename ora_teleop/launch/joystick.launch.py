import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
  pkg_share = get_package_share_directory('ora_teleop')

  joystick_params = os.path.join(pkg_share, 'config', 'joystick.yaml')

  joystick_node = Node(
    package='joy',
    executable='joy_node',
    parameters=[joystick_params],
  )
  
  teleop_node = Node(
    package='teleop_twist_joy',
    executable='teleop_node',
    name='teleop_node',
    parameters=[joystick_params],
    remappings=[
      ('/cmd_vel', '/cmd_vel_joy'),
    ],
  )

  return LaunchDescription([
    joystick_node,
    teleop_node,
  ])

