from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
import os
import launch.actions


def generate_launch_description():
    pkg_share = get_package_share_directory('ora_navigation')
    #ekf_params = os.path.join(pkg_share, 'config/dual_ekf.yaml')
    #navsat_params = os.path.join(pkg_share, 'config/navsat_params.yaml')
    gps_ekf_params = os.path.join(pkg_share, 'config/gps_ekf.yaml')

    return LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                "output_final_position", default_value="false"
            ),
            launch.actions.DeclareLaunchArgument(
                "output_location", default_value="~/dual_ekf_navsat_example_debug.txt"
            ),
            launch_ros.actions.Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_odom",
                output="screen",
                parameters=[gps_ekf_params, {"use_sim_time": True}],
                # published
                remappings=[("odometry/filtered", "odometry/local")],
            ),
            launch_ros.actions.Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_map",
                output="screen",
                parameters=[gps_ekf_params, {"use_sim_time": True}],
                # published
                remappings=[("odometry/filtered", "odometry/global")],
            ),
            launch_ros.actions.Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform",
                output="screen",
                parameters=[gps_ekf_params, {"use_sim_time": True}],
                remappings=[
                    #subscriptions
                    ("imu/data", "/zed/zed_node/imu/data"), #IMU from ZED
                    ("odometry/filtered", "odometry/global"), #subscribe to odometry/global from ekf_filter_node_map
                    ("fix", "gps/fix"), #Ublox gps
                    #published
                    ("odometry/gps", "odometry/gps"),
                    ("gps/filtered", "gps/filtered"),
                ],
            ),
        ]
    )