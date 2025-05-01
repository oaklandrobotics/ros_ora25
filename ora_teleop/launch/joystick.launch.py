import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
  pkg_share = get_package_share_directory('ora_teleop')

  use_bt = LaunchConfiguration('use_bt')

  bt_joystick_params = os.path.join(pkg_share, 'config', 'bt_joy.yaml')
  wired_joystick_params = os.path.join(pkg_share, 'config', 'wired_joy.yaml')

  # Create nodes for both BT and Wired usage
  bt_joystick_node = Node(
    package='joy',
    executable='joy_node',
    parameters=[bt_joystick_params],
    condition=IfCondition(use_bt)
  )

  wired_joystick_node = Node(
    package='joy',
    executable='joy_node',
    parameters=[wired_joystick_params],
    condition=UnlessCondition(use_bt)
  )
  
  bt_teleop_node = Node(
    package='teleop_twist_joy',
    executable='teleop_node',
    name='teleop_node',
    parameters=[bt_joystick_params],
    remappings=[
      ('/cmd_vel', '/cmd_vel_joy'),
    ],
    condition=IfCondition(use_bt)
  )
  
  wire_teleop_node = Node(
    package='teleop_twist_joy',
    executable='teleop_node',
    name='teleop_node',
    parameters=[wired_joystick_params],
    remappings=[
      ('/cmd_vel', '/cmd_vel_joy'),
    ],
    condition=UnlessCondition(use_bt)
  )
  
  reinit_control_node = Node(
    package='ora_teleop',
    executable='reinit_node',
    output='screen'
  )

  return LaunchDescription([
    DeclareLaunchArgument(
      'use_bt',
      default_value='true',
      description='Whether to use Bluetooth controller or wired controller.'
    ),
    
    bt_joystick_node,
    wired_joystick_node,
    
    bt_teleop_node,
    wire_teleop_node,
    
    reinit_control_node,
  ])

