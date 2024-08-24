import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    pkg_name = 'turbopi_ros'
    pkg_path = os.path.join(get_package_share_directory(pkg_name))
    nav2_params_file = os.path.join(pkg_path, 'config', 'nav2_params.yaml')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'),
                        'launch'),
                        '/navigation_launch.py'
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file' : nav2_params_file,
        }.items(),
    )

    return LaunchDescription([
        nav2,
    ])
