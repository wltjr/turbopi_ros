import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def launch_setup(context: LaunchContext):

    filename = 'turbopi.urdf.xacro'
    pkg_name = 'turbopi_ros'
    pkg_path = os.path.join(get_package_share_directory(pkg_name))
    world_file = context.perform_substitution(LaunchConfiguration('world')) + '.world'
    world = os.path.join(pkg_path,'worlds', world_file)
    xacro_file = os.path.join(pkg_path,'description',filename)

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_file,
            " ",
            "use_hardware:=ign_gazebo",
            " ",
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
                    launch_arguments={
                        'pause' : 'true',
                        'gz_args' : world,
                    }.items(),
                )

    create_entity = Node(package='ros_gz_sim',
                        executable='create',
                        arguments=['-topic', '/robot_description',
                                    '-entity', 'robot'],
                        output='screen')
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
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
        arguments=["diff_drive_controller", "-c", "/controller_manager"],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    position_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controllers", "-c", "/controller_manager"],
    )

    static_transform_publisher_odom = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments= ["--frame-id", "base_link",
                        "--child-frame-id", "turbopi_ros/odom"]
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=create_entity,
            on_exit=[joint_state_broadcaster],
        )
    )

    delayed_static_transform_publisher_odom = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=create_entity,
            on_exit=[static_transform_publisher_odom],
        )
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[diff_drive_spawner],
        )
    )

    delayed_position_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[position_spawner],
        )
    )

    nodes = [
        gazebo,
        create_entity,
        bridge,
        node_robot_state_publisher,
        delayed_joint_broad_spawner,
        delayed_static_transform_publisher_odom,
        delayed_diff_drive_spawner,
        delayed_position_spawner,
    ]

    return nodes

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "world",
            default_value="empty",
            description="The world the robot will be spawned within.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
