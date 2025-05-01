import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
  # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
  pkg_share = get_package_share_directory('ora_description')
  
  world_path=os.path.join(pkg_share, 'worlds/igvc_world.sdf')

  rsp = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      os.path.join(
        pkg_share,'launch','rsp.launch.py'
      )
    ]),
    launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
  )

  # Launch Gazebo with the standard ROS‑provided launch file
  gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(
          get_package_share_directory('gazebo_ros'),
          'launch', 'gazebo.launch.py'
      )
    ),
    launch_arguments={'world': world_path}.items()
  )

  # Spawn robot
  spawn_entity = Node(
      package='gazebo_ros',
      executable='spawn_entity.py',
      arguments=['-entity', 'horizon', '-topic', 'robot_description'],
      output='screen'
  )
  
  # Delay spawn by 5 seconds
  # Was running into race condition where spawn_entity was starting before gazebo
  delayed_spawn = TimerAction(
    period=5.0,
    actions=[ spawn_entity ]
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

  # Launch rsp, gz, and spawn the bot in gz
  return LaunchDescription([
    rsp,
    gazebo,
    delayed_spawn,
    spawn_diff_drive,
    spawn_joint_broadcaster
  ])