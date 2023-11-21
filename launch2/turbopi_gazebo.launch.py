import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg_name = 'turbopi_ros'
    pkg_path = os.path.join(get_package_share_directory(pkg_name))

    controller_params = os.path.join(
        pkg_path,
        'config',
        'turbopi_controllers.yaml'
    )

    launch_gazebo_arg = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('ros_ign_gazebo'), '/launch', '/ign_gazebo.launch.py']),
        launch_arguments={
           'ign_args' : os.path.join(pkg_path,'worlds','empty.sdf')
        }.items(),
    )

    node_joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
    )

    node_spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=["-topic", "robot_description"],
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_params],
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    return LaunchDescription([
        launch_gazebo_arg,
        node_joint_state_publisher,
        node_spawn_robot,
        controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner
    ])
