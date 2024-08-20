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

    CM = "/controller_manager"
    custom = eval(context.perform_substitution(LaunchConfiguration('custom')).title())
    filename = 'turbopi.urdf.xacro'
    pkg_name = 'turbopi_ros'
    pkg_path = os.path.join(get_package_share_directory(pkg_name))
    slam_params_file = os.path.join(pkg_path, 'config', 'slam_toolbox.yaml')
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
            "use_style:=",
            "depth" if custom else "default",
            " ",
            "use_version:=",
            "6" if os.path.isdir("/opt/ros/iron/") else "8",
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

    create_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description',
                    '-entity', 'robot'],
        output='screen'
    )

    bridge_args = [
        '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
        '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        '/sonar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
    ]
    if custom:
        bridge_args += [
            '/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ]

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_args,
    )

    image_bridge_args = ['/camera']
    if custom:
        image_bridge_args += ['/depth_camera']

    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=image_bridge_args,
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    joint_state_broadcaster = Node(
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
        output='screen',
        parameters=[ slam_params_file, {'use_sim_time': True} ],
        remappings=[('/map', '/slam_toolbox/map'),],
    )

    static_transform_publisher_camera = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments= ["--frame-id", "base_link",
                        "--child-frame-id", "turbopi/base_link/camera"]
    )

    static_transform_publisher_depth_camera = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments= ["--frame-id", "base_link",
                        "--child-frame-id", "turbopi/base_link/depth_camera"]
    )

    static_transform_publisher_lidar = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments= ["--frame-id", "base_link",
                        "--child-frame-id", "turbopi/base_link/lidar_sensor"]
    )

    static_transform_publisher_odom = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments= ["--frame-id", "base_link",
                        "--child-frame-id", "turbopi/odom"]
    )

    static_transform_publisher_sonar = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments= ["--frame-id", "base_link",
                        "--child-frame-id", "turbopi/base_link/sonar_sensor"]
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=create_entity,
            on_exit=[joint_state_broadcaster],
        )
    )

    delayed_static_transform_publisher_camera = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=create_entity,
            on_exit=[static_transform_publisher_camera],
        )
    )

    delayed_static_transform_publisher_depth_camera = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=create_entity,
            on_exit=[static_transform_publisher_depth_camera],
        )
    )

    delayed_static_transform_publisher_lidar = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=create_entity,
            on_exit=[static_transform_publisher_lidar],
        )
    )

    delayed_static_transform_publisher_odom = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=create_entity,
            on_exit=[static_transform_publisher_odom],
        )
    )

    delayed_static_transform_publisher_sonar = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=create_entity,
            on_exit=[static_transform_publisher_sonar],
        )
    )

    delayed_slam_toolbox_node_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=create_entity,
            on_exit=[slam_toolbox_node],
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
        image_bridge,
        node_robot_state_publisher,
        delayed_joint_broad_spawner,
    ]

    # Enable features for custom robot style 3d depth camera or 2d camera
    if custom:
        nodes += [ delayed_static_transform_publisher_depth_camera ]
    else:
        nodes += [
            delayed_static_transform_publisher_camera,
            delayed_position_spawner,
        ]

    nodes += [
        delayed_static_transform_publisher_lidar,
        delayed_static_transform_publisher_odom,
        delayed_static_transform_publisher_sonar,
        delayed_slam_toolbox_node_spawner,
    ]

    return nodes

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "custom",
            default_value="True",
            description="Run customized 3d camera vs sonar with 2d camera.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "world",
            default_value="empty",
            description="The world the robot will be spawned within.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
