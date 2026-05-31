import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    pkg_name = 'turbopi_ros'
    pkg_path = os.path.join(get_package_share_directory(pkg_name))
    nav2_params_file = os.path.join(pkg_path, 'config', 'nav2_params.yaml')

    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='Use simulation (Gazebo) clock. Set true only when running in Gazebo.',
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'),
                        'launch'),
                        '/navigation_launch.py'
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('sim'),
            'params_file' : nav2_params_file,
        }.items(),
    )

    return LaunchDescription([
        sim_arg,
        nav2,
    ])
