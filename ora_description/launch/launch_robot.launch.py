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
  # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
  pkg_share = get_package_share_directory('ora_description')
  lidar_pkg_share = get_package_share_directory('sllidar_ros2')
  zed_pkg_share = get_package_share_directory('zed_wrapper')

  # Launch parameters
  lidar_serial_port = LaunchConfiguration('lidar_serial_port')

  rsp = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      os.path.join(
        pkg_share,'launch','rsp.launch.py'
      )
    ]),
    launch_arguments={
      'use_sim_time'    : 'false',
      'use_ros2_control': 'true'
    }.items()
  )

  robot_description = Command(['ros2 ', 'param ', 'get ', '--hide-type ', '/robot_state_publisher ', 'robot_description'])
  
  controller_params_file = os.path.join(pkg_share, 'config', 'my_controllers.yaml')

  ##############################################
  #                                            #
  #                   SENSORS                  #
  #                                            #
  ##############################################

  # RPLidar
  # Default params per the launch file
  # ('channel_type',     default='serial')
  # ('serial_port',      default='/dev/ttyUSB0')
  # ('serial_baudrate',  default='115200')
  # ('frame_id',         default='laser')
  # ('inverted',         default='false')
  # ('angle_compensate', default='true')
  # ('scan_mode',        default='Sensitivity')
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
      'frame_id'        : 'laser',
      'inverted'        : 'false',
      'angle_compensate': 'true',
      'scan_mode'       : 'Sensitivity'
    }.items()
  )

  delayed_lidar = TimerAction(
    period=2.5,
    actions=[rplidar_node]
  )

  # Zed
  # Default params per the launch file
  # svo_path = LaunchConfiguration('svo_path')
  # use_sim_time = LaunchConfiguration('use_sim_time')
  # sim_mode = LaunchConfiguration('sim_mode')
  # sim_address = LaunchConfiguration('sim_address')
  # sim_port = LaunchConfiguration('sim_port')
  # stream_address = LaunchConfiguration('stream_address')
  # stream_port = LaunchConfiguration('stream_port')
  # container_name = LaunchConfiguration('container_name')
  # namespace = LaunchConfiguration('namespace')
  # camera_name = LaunchConfiguration('camera_name')
  # camera_model = LaunchConfiguration('camera_model')
  # node_name = LaunchConfiguration('node_name')
  # ros_params_override_path = LaunchConfiguration('ros_params_override_path')
  # config_ffmpeg = LaunchConfiguration('ffmpeg_config_path')
  # serial_number = LaunchConfiguration('serial_number')
  # camera_id = LaunchConfiguration('camera_id')
  # publish_urdf = LaunchConfiguration('publish_urdf')
  # publish_tf = LaunchConfiguration('publish_tf')
  # publish_map_tf = LaunchConfiguration('publish_map_tf')
  # publish_imu_tf = LaunchConfiguration('publish_imu_tf')
  # xacro_path = LaunchConfiguration('xacro_path')
  # custom_baseline = LaunchConfiguration('custom_baseline')
  # enable_gnss = LaunchConfiguration('enable_gnss')
  # gnss_antenna_offset = LaunchConfiguration('gnss_antenna_offset')
  zed_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      os.path.join(
        zed_pkg_share, 'launch', 'zed_camera.launch.py'
      )
    ]),
    launch_arguments={
      'camera_model': 'zed2i',
      'sim_mode': 'false',
      'use_sim_time': 'false',
    }.items()
  )

  ##############################################
  #                                            #
  #                ROS2 Control                #
  #                                            #
  ##############################################

  controller_manager = Node(
      package='controller_manager',
      executable='ros2_control_node',
      parameters=[
        {'robot_description': robot_description},
        controller_params_file
      ],
  )
  
  # Controller node didnt like it when it started up before some other nodes
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

  # Launch rsp, gz, and spawn the bot in gz
  return LaunchDescription([
    # Launch Parameters
    DeclareLaunchArgument(
      'lidar_serial_port',
      default_value='/dev/ttyUSB0',
      description='Serial port that Lidar is connected to'),
    
    # Start robot state publisher
    rsp,
    
    # Start sensors
    delayed_lidar,
    # zed_node,
    
    # Start ROS2 Control
    delayed_controller_manager,
    delayed_diff_drive,
    delayed_joint_broadcaster
  ])