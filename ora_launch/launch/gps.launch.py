import launch, os
import launch.launch_description_sources
import launch_ros
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ublox = get_package_share_directory('ublox_dgnss')
    mdot_pw = os.getenv('MDOT_PW')

    ublox_driver = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(ublox, 'launch', 'ublox_rover_hpposllh_navsatfix.launch.py')
        )
    )

    ntrip_client = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(ublox, 'launch', 'ntrip_client.launch.py'),
        ),
        launch_arguments={
            'host': '148.149.0.87',
            'port': '10001',
            'mountpoint': 'METR_RTCM3-GG',
            'username': 'dmocnik',
            'password': mdot_pw,
            'use_https': 'False'
        }.items()
    )

    ld = launch.LaunchDescription()


    ld.add_action(ublox_driver)
    ld.add_action(ntrip_client)

    return ld