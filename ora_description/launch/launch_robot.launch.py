import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction, RegisterEventHandler
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.event_handlers import OnProcessStart

def generate_launch_description():
  # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
  pkg_share = get_package_share_directory('ora_description')

  rsp = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      os.path.join(
        pkg_share,'launch','rsp.launch.py'
      )
    ]),
    launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
  )

  robot_description = Command(['ros2', 'param', 'get', '--hide-type', '/robot_state_publisher', 'robot_description'])
  
  controller_params_file = os.path.join(pkg_share, 'config', 'my_controllers.yaml')

  controller_manager = Node(
      package='controller_manager',
      executable='ros2_control_node',
      parameters=[
        {'robot_description': robot_description},
        controller_params_file
      ],
  )
  
  delayed_controller_manager = TimerAction(
    period=2.5,
    actions=[controller_manager]
  )

  # Spawn diff drive controller and joint broadcaster from ros2_control
  # This is necessary to get the robot to move
  spawn_diff_drive = Node(
      package='controller_manager',
      executable='spawner',
      arguments=['diff_drive_cont'],
      output='screen'
  )
  
  delayed_diff_drive = RegisterEventHandler(
    event_handler=OnProcessStart(
      target_action=controller_manager,
      on_start=[spawn_diff_drive]
    )
  )
  
  spawn_joint_broadcaster = Node(
      package='controller_manager',
      executable='spawner',
      arguments=['joint_state_broadcaster'],
      output='screen'
  )
  
  delayed_joint_broadcaster = RegisterEventHandler(
    event_handler=OnProcessStart(
      target_action=controller_manager,
      on_start=[spawn_joint_broadcaster]
    )
  )

  # Launch rsp, gz, and spawn the bot in gz
  return LaunchDescription([
    rsp,
    delayed_controller_manager,
    delayed_diff_drive,
    delayed_joint_broadcaster
  ])