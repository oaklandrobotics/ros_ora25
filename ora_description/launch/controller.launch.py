import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    pkg_share = get_package_share_directory('ora_description')

    robot_description = Command(['ros2 ', 'param ', 'get ', '--hide-type ', '/robot_state_publisher ', 'robot_description'])

    controller_params_file = os.path.join(pkg_share, 'config', 'my_controllers.yaml')

    controller_manager = Node(
      package='controller_manager',
      executable='ros2_control_node',
      parameters=[
        {'robot_description': robot_description},
        controller_params_file
      ],
  )

    return LaunchDescription([controller_manager])