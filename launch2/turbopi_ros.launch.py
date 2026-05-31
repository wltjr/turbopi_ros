import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_context import LaunchContext
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _mecanum_drive_available() -> bool:
    """Return True if mecanum_drive_controller plugin is installed on this system."""
    try:
        import subprocess as sp
        result = sp.run(
            ["ros2", "pkg", "prefix", "mecanum_drive_controller"],
            capture_output=True, timeout=5,
        )
        return result.returncode == 0
    except Exception:
        return False


def launch_setup(context: LaunchContext):

    CM = "/controller_manager"
    pkg_name = 'turbopi_ros'
    filename = 'turbopi.urdf.xacro'

    pkg_path = os.path.join(get_package_share_directory(pkg_name))
    camera = eval(context.perform_substitution(LaunchConfiguration('camera')).title())
    camera_type = context.perform_substitution(LaunchConfiguration('camera_type'))
    debug = eval(context.perform_substitution(LaunchConfiguration('debug')).title())
    drive = context.perform_substitution(LaunchConfiguration('drive'))
    lidar = eval(context.perform_substitution(LaunchConfiguration('lidar')).title())
    lidar_port = context.perform_substitution(LaunchConfiguration('lidar_port'))
    lidar_yaw = context.perform_substitution(LaunchConfiguration('lidar_yaw'))
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
            "use_drive:=",
            drive,
            " ",
            "use_camera:=",
            camera_type,
            " ",
            "use_lidar:=",
            "lidar" if lidar else "default",
            " ",
            "lidar_yaw:=",
            lidar_yaw,
            " ",
        ]
    )
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_params],
        output='both',
        ros_arguments=['--log-level', 'Board:=debug'] if debug else [],
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
        arguments=[
            "diff_drive_controller", "-c", CM,
            "--controller-ros-args",
            "-r /diff_drive_controller/cmd_vel:=/cmd_vel",
            "--controller-ros-args",
            "-r /diff_drive_controller/odom:=/odom",
        ],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", CM],
    )

    mecanum_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mecanum_drive_controller", "-c", CM,
            "--controller-ros-args",
            "-r /mecanum_drive_controller/tf_odometry:=/tf",
            "--controller-ros-args",
            "-r /mecanum_drive_controller/reference:=/cmd_vel",
            "--controller-ros-args",
            "-r /mecanum_drive_controller/odometry:=/odom",
        ],
    )

    if drive == "mecanum":
        if _mecanum_drive_available():
            drive_spawner = mecanum_drive_spawner
        else:
            import sys
            print(
                "\n[WARNING] drive:=mecanum requested but mecanum_drive_controller is NOT "
                "installed on this system (not available in your ros2_controllers build).\n"
                "          Falling back to diff_drive_controller.\n"
                "          To install: sudo apt install ros-jazzy-mecanum-drive-controller\n",
                file=sys.stderr,
            )
            drive_spawner = diff_drive_spawner
    else:
        drive_spawner = diff_drive_spawner

    position_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controllers", "-c", CM],
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        parameters=[ slam_params_file, {'use_sim_time': sim} ],
        output='screen',
    )

    # slam_toolbox 2.8.4 (Jazzy) auto-configures but does NOT auto-activate.
    # A lifecycle_manager with autostart=True sends configure→activate automatically.
    # It is launched 5 s after slam_toolbox starts (giving the node time to finish
    # on_configure() before the manager sends the activate transition).
    lifecycle_manager_slam = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'use_sim_time': sim,
            'autostart': True,
            'node_names': ['slam_toolbox'],
            # Increase bond timeout: slam_toolbox can be slow to respond during
            # startup while it waits for TF. Default 4s is too short.
            'bond_timeout': 10.0,
        }],
    )

    battery_monitor_node = Node(
        package='turbopi_ros',
        executable='battery_monitor_node',
        parameters=[],
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

    # RPLidar driver node – publishes /scan topic and provides /start_motor + /stop_motor services
    # Default serial port is /dev/rplidar (udev symlink). Override with lidar_port:=/dev/ttyUSB0
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': lidar_port,
            'serial_baudrate': 115200,
            'frame_id': 'lidar',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }],
    )

    # NOTE: rplidar_node auto-starts the motor on init.
    # The /start_motor service call is NOT needed and actually causes
    # a Stop→Start cycle that interrupts scanning.

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

    delayed_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_broad_spawner,
            on_exit=[drive_spawner],
        )
    )

    # Chain position_controllers after drive_spawner exits (not after joint_broad_spawner)
    # because two OnProcessExit handlers on the same target only fire one reliably.
    delayed_position_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=drive_spawner,
            on_exit=[position_spawner],
        )
    )

    # RPLidar starts after position_controllers spawner exits (chains cleanly).
    # When camera_type != 'default', position_spawner is not started, so rplidar
    # is chained after drive_spawner instead (handled in nodes list below).
    delayed_rplidar_spawner_after_position = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=position_spawner,
            on_exit=[rplidar_node],
        )
    )

    # Fallback: rplidar after drive_spawner when position_controllers is not used.
    delayed_rplidar_spawner_after_drive = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=drive_spawner,
            on_exit=[rplidar_node],
        )
    )

    # Give rplidar_node 3 s to fully initialise, then start slam_toolbox.
    # After a further 5 s (slam_toolbox needs ~3 s to finish on_configure),
    # start the lifecycle_manager which auto-activates slam_toolbox.
    # No /start_motor call needed – rplidar_node auto-starts the motor on init.
    delayed_slam_toolbox_node_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=rplidar_node,
            on_start=[
                TimerAction(period=3.0, actions=[slam_toolbox_node]),
                TimerAction(period=8.0, actions=[lifecycle_manager_slam]),
            ],
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

    # v4l2 camera: chain after position_spawner when default camera (servos active),
    # otherwise chain after drive_spawner.
    delayed_v4l2_camera_node_after_position = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=position_spawner,
            on_exit=[v4l2_camera_node],
        )
    )

    delayed_v4l2_camera_node_after_drive = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=drive_spawner,
            on_exit=[v4l2_camera_node],
        )
    )

    nodes = [
        battery_monitor_node,
        battery_node,
        controller_manager,
        node_robot_state_publisher,
        delayed_joint_broad_spawner,
        delayed_drive_spawner,
        delayed_infrared_node_spawner,
        delayed_sonar_node_spawner,
    ]

    _add_optional_nodes(
        nodes,
        camera_type=camera_type,
        camera=camera,
        lidar=lidar,
        delayed_position_spawner=delayed_position_spawner,
        delayed_v4l2_camera_node_after_position=delayed_v4l2_camera_node_after_position,
        delayed_v4l2_camera_node_after_drive=delayed_v4l2_camera_node_after_drive,
        delayed_rplidar_spawner_after_position=delayed_rplidar_spawner_after_position,
        delayed_rplidar_spawner_after_drive=delayed_rplidar_spawner_after_drive,
        delayed_slam_toolbox_node_spawner=delayed_slam_toolbox_node_spawner,
    )

    return nodes


def _add_optional_nodes(
    nodes,
    camera_type,
    camera,
    lidar,
    delayed_position_spawner,
    delayed_v4l2_camera_node_after_position,
    delayed_v4l2_camera_node_after_drive,
    delayed_rplidar_spawner_after_position,
    delayed_rplidar_spawner_after_drive,
    delayed_slam_toolbox_node_spawner,
):
    """Add optional controller/sensor nodes using a conflict-free OnProcessExit chain.

    Chain: drive -> [position] -> [rplidar] -> [slam]
    Each step exits before the next starts, avoiding duplicate event-handler targets.
    v4l2_camera_node is long-running (never exits) so rplidar can share the same
    position_spawner exit event without conflict.
    """
    use_default_camera = camera_type == 'default'

    if use_default_camera:
        # position_controllers (pan/tilt) only exist for the 2DOF camera mount.
        nodes += [delayed_position_spawner]
        if camera:
            nodes += [delayed_v4l2_camera_node_after_position]
        if lidar:
            # v4l2_camera_node never exits, so rplidar can safely share the same
            # position_spawner OnProcessExit trigger without conflict.
            nodes += [delayed_rplidar_spawner_after_position, delayed_slam_toolbox_node_spawner]
    else:
        # No position_controllers: chain camera and lidar directly after drive_spawner.
        if camera:
            nodes += [delayed_v4l2_camera_node_after_drive]
        if lidar:
            nodes += [delayed_rplidar_spawner_after_drive, delayed_slam_toolbox_node_spawner]


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "camera",
            default_value="False",
            description="Start with v4l2_camera node (requires v4l2_camera package)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "debug",
            default_value="False",
            description="Enable Board DEBUG logging in ros2_control_node (shows raw UART packets)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "camera_type",
            default_value="depth",
            description="Camera type: 'default' (2DOF pan/tilt) or 'depth' (Orbbec Astra S). "
                        "Use 'default' to enable position_controllers for pan/tilt servos.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "drive",
            default_value="diff",
            description="Drive system diff or mecanum",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "lidar",
            default_value="False",
            description="Start with lidar hardware",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "lidar_port",
            default_value="/dev/rplidar",
            description="Serial port for RPLidar (e.g. /dev/rplidar or /dev/ttyUSB0)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "lidar_yaw",
            default_value="3.14159265358979",
            description="Lidar mount yaw rotation in radians (default pi=3.14159 for 180° "
                        "backward-facing mount). Use 0.0 for forward-facing mount.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim",
            default_value="False",
            description="Start with simulated mock hardware",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
