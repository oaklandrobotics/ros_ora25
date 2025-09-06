import launch, os, launch.launch_description_sources
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    zed = get_package_share_directory('zed_wrapper')
    zed_params = os.path.join(get_package_share_directory('ora_launch'), "config", "zed.yaml")

    zed_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(zed, 'launch', 'zed_camera.launch.py')
        ),
        launch_arguments={
            'camera_model': 'zed2i',
            'ros_params_override_path': zed_params
        }.items()
    )

    ld = launch.LaunchDescription()
    ld.add_action(zed_launch)
    return ld