import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    pkg_name = 'turbopi_ros'
    filename = 'turbopi.urdf.xacro'

    pkg_path = os.path.join(get_package_share_directory(pkg_name))
    xacro_file = os.path.join(pkg_path,'description',filename)
    robot_description_config = xacro.process_file(xacro_file)

    params = {'robot_description': robot_description_config.toxml()}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    return LaunchDescription([
        node_robot_state_publisher
    ])
