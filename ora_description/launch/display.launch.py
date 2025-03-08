from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare(package='ora_description').find('ora_description')
    default_model_path = os.path.join(pkg_share, 'src', 'description', 'horizon_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'config.rviz')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', LaunchConfiguration('model')]),
                value_type=str
            )
        },
        {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    #
    # Handled by the ora_gazebo package now
    #

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )
    # joint_state_publisher_gui_node = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui',
    #     condition=IfCondition(LaunchConfiguration('gui'))
    # )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time' )}],
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'horizon', '-topic', 'robot_description'],
        output='screen'
    )

    return LaunchDescription([
        # DeclareLaunchArgument(name='gui', default_value='True', description='Flag to enable joint_state_publisher_gui'),
        DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot model file'),
        DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Flag to enable use_sim_time'),
        #
        # Handled by the ora_gazebo package now
        #
        # DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'),
        ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'], output='screen'),
        joint_state_publisher_node,
        # joint_state_publisher_gui_node,
        robot_state_publisher_node,
        spawn_entity,
        rviz_node
    ])