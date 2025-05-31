import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node

from nav2_common.launch import RewrittenYaml

def generate_launch_description():
  pkg_share = get_package_share_directory("ora_navigation")
  nav2_pkg_share = get_package_share_directory("nav2_bringup")
  config = os.path.join(pkg_share, "config")
  nav2_params = os.path.join(config, "nav2_params.yaml")
  configured_params = RewrittenYaml(
      source_file=nav2_params, root_key="", param_rewrites="", convert_types=True
  )
  
  use_sim_time = LaunchConfiguration('use_sim_time')
  use_rviz = LaunchConfiguration('use_rviz')

  navigation2_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(nav2_pkg_share, "launch", "navigation_launch.py")
    ),
    launch_arguments={
      "use_sim_time": use_sim_time,
      "params_file": configured_params,
      "autostart": "True",
    }.items(),
    )

  rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg_share, "launch", "rviz_launch.py")
        ),
        condition=IfCondition(use_rviz),
    )

  return LaunchDescription([
    DeclareLaunchArgument(
      'use_sim_time',
      default_value='false',
      description='Use sim time if true'),
    DeclareLaunchArgument(
      'use_rviz',
      default_value='true',
      description='Show Rviz if true'
    ),
    navigation2_cmd,
    rviz_cmd
  ])

