import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchService, LaunchContext
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, RegisterEventHandler
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.event_handlers import OnProcessStart
from launch.events.process import ProcessStarted

def generate_launch_description():
  # Packages for launching
  launch_pkg_share = get_package_share_directory('ora_launch')
  desc_pkg_share = get_package_share_directory('ora_description')
  
  # Robot Description
  robot_description = Command([
    'ros2 ', 'param ', 'get ', '--hide-type ', '/robot_state_publisher ', 'robot_description'
  ])
  
  # Params Files
  controller_params_file = os.path.join(desc_pkg_share, 'config', 'my_controllers.yaml')

  controller_manager = Node(
    package='controller_manager',
    executable='ros2_control_node',
    parameters=[
      {'robot_description': robot_description},
      controller_params_file
    ],
  )
  
  # Spawn diff drive controller and joint broadcaster from ros2_control
  # This is necessary to get the robot to move
  spawn_diff_drive = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['diff_drive_cont'],
    output='screen'
  )
  
  spawn_joint_broadcaster = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['joint_state_broadcaster'],
    output='screen'
  )
  
  # Same deal as above, dont start our controllers until the controller manager is started
  delayed_diff_drive = RegisterEventHandler(
    event_handler=OnProcessStart(
      target_action=controller_manager,
      on_start=[spawn_diff_drive]
    )
  )
  
  delayed_joint_broadcaster = RegisterEventHandler(
    event_handler=OnProcessStart(
      target_action=controller_manager,
      on_start=[spawn_joint_broadcaster]
    )
  )
  
  # Controller node didnt like it when it started up before some other nodes
  delayed_controller_manager = TimerAction(
    period=2.5,
    actions=[controller_manager]
  )
  
  # Twist Multiplexer
  twist_mux = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      os.path.join(
        launch_pkg_share, 'launch', 'twist_mux.launch.py'
      )
    ]),
    launch_arguments = {
      'use_sim_time' : 'false'
    }.items()
  )
  
  return LaunchDescription([
    delayed_controller_manager,
    delayed_diff_drive,
    delayed_joint_broadcaster,
    twist_mux
  ])