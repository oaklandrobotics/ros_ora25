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
  auton_pkg_share = get_package_share_directory('ora_auton')
  nav_pkg_share = get_package_share_directory('ora_navigation')
  lidar_pkg_share = get_package_share_directory('sllidar_ros2')
  zed_pkg_share = get_package_share_directory('zed_wrapper')
  

   
  ##############################################
  #                                            #
  #                     RSP                    #
  #                                            #
  ##############################################
  rsp = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      os.path.join(
        desc_pkg_share, 'launch', 'rsp.launch.py'
      )
    ]),
    launch_arguments={
      'use_sim_time' : 'false',
      'use_ros2_control' : 'true'
    }.items()
  )
  
  ##############################################
  #                                            #
  #                ROS2 Control                #
  #                                            #
  ##############################################
  ros2_control = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      os.path.join(
        launch_pkg_share, 'launch', 'control.launch.py'
      )
    ])
  )
  
  ##############################################
  #                                            #
  #                  Sensors                   #
  #                                            #
  ##############################################
  # Lidar
  filtered_lidar = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      os.path.join(
        launch_pkg_share, 'launch', 'filtered_lidar.launch.py'
      )
    ]),
  )
  
  # ZED 
  custom_zed = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      os.path.join(
        launch_pkg_share, 'launch', 'custom_zed.launch.py'
      )
    ]),
  )
  
  # GPS
  ublox_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      os.path.join(
        launch_pkg_share, 'launch', 'gps.launch.py'
      )
    ]),
  )
  
  ##############################################
  #                                            #
  #              Autonomous Stuff              #
  #                                            #
  ##############################################
  # EKF
  dual_ekf_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      os.path.join(
        nav_pkg_share, 'launch', 'dual_ekf_navsat.launch.py'
      )
    ]),
  )
  
  delayed_ekf = TimerAction(
    period=5.0,
    actions=[dual_ekf_node]
  )
  
  goal_publisher_node = Node(
    package='ora_auton',
    executable='goal_publisher'
  )
  
  stacklight_service_node = Node(
    package='ora_auton',
    executable='stacklight_service'
  )
  
  line_percep_node = Node(
    package='line_perception',
    executable='line_perception'
  )
  
  nav2 = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      os.path.join(
        nav_pkg_share,'launch','nav2.launch.py'
      )
    ])
  )
  
  delayed_nav2 = TimerAction(
    period=15.0,
    actions=[nav2]
  )
  
  return LaunchDescription([    
    # Start the robot state publisher
    rsp,
    
    # Start ros2_control
    ros2_control,
    
    # Start all of the sensors
    filtered_lidar,
    custom_zed,
    ublox_node,
    
    # Start Autonomous things
    delayed_ekf,
    stacklight_service_node,
    line_percep_node,
    delayed_nav2
  ])