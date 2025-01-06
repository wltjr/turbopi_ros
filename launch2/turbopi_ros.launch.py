import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart, OnShutdown
from launch.launch_context import LaunchContext
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

import subprocess

def launch_setup(context: LaunchContext):

    CM = "/controller_manager"
    pkg_name = 'turbopi_ros'
    filename = 'turbopi.urdf.xacro'

    pkg_path = os.path.join(get_package_share_directory(pkg_name))
    sim = eval(context.perform_substitution(LaunchConfiguration('sim')).title())
    camera_params_file = os.path.join(pkg_path, 'config', 'camera.yaml')
    slam_params_file = os.path.join(pkg_path, 'config', 'slam_toolbox.yaml')
    controller_params = os.path.join(pkg_path, 'config', 'turbopi_controllers.yaml')
    xacro_file = os.path.join(pkg_path,'description',filename)

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_file,
            " ",
            "use_hardware:=",
            "mock" if sim else "robot",
            " ",
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_params],
        output='both',
        remappings=[
            ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
            ("/diff_drive_controller/odom", "/odom"),
        ],
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "-c", CM],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", CM],
    )

    position_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controllers", "-c", CM],
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        parameters=[ slam_params_file, {'use_sim_time': True} ],
    )

    battery_node = Node(
        package='turbopi_ros',
        executable='battery_node',
        parameters=[],
    )

    infrared_node = Node(
        package='turbopi_ros',
        executable='infrared_node',
        parameters=[],
    )

    sonar_node = Node(
        package='turbopi_ros',
        executable='sonar_node',
        parameters=[],
    )

    start_lidar = ExecuteProcess(
        cmd=[
            [
                FindExecutable(name="ros2"),
                " service call ",
                "/start_motor ",
                "std_srvs/srv/Empty",
            ]
        ],
        shell=True,
    )

    v4l2_camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        parameters=[camera_params_file],
        remappings=[('/image_raw', '/camera'),],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_broad_spawner,
            on_exit=[diff_drive_spawner],
        )
    )

    delayed_position_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_broad_spawner,
            on_exit=[position_spawner],
        )
    )

    delayed_slam_toolbox_node_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_broad_spawner,
            on_exit=[start_lidar, slam_toolbox_node],
        )
    )

    delayed_infrared_node_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_broad_spawner,
            on_start=[infrared_node],
        )
    )

    delayed_sonar_node_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_broad_spawner,
            on_start=[sonar_node],
        )
    )

    delayed_v4l2_camera_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_broad_spawner,
            on_exit=[v4l2_camera_node],
        )
    )

    stop_lidar_on_shutdown = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=slam_toolbox_node,
            on_exit=[
                LogInfo(msg='Stopping lidar'),
                OpaqueFunction(function=stop_lidar),
                LogInfo(msg='Stopped lidar'),
            ],
        )
    )

    nodes = [
        battery_node,
        controller_manager,
        node_robot_state_publisher,
        delayed_joint_broad_spawner,
        delayed_diff_drive_spawner,
        delayed_position_spawner,
        delayed_slam_toolbox_node_spawner,
        delayed_infrared_node_spawner,
        delayed_sonar_node_spawner,
        delayed_v4l2_camera_node,
        stop_lidar_on_shutdown,
    ]

    return nodes


def stop_lidar(context: LaunchContext):
    subprocess.run("ros2 service call /stop_motor std_srvs/srv/Empty", shell=True)


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim",
            default_value="False",
            description="Start with simulated mock hardware",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
