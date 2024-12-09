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
    dtl_params_file = os.path.join(pkg_path, 'config', 'depthimage_to_laserscan.yaml')
    ptl_params_file = os.path.join(pkg_path, 'config', 'pointcloud_to_laserscan.yaml')
    rlsm_params_file = os.path.join(pkg_path, 'config', 'ros2_laser_scan_merger.yaml')
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
        arguments=['-topic', '/robot_description'],
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

    bridge_remappings = []
    if custom:
        bridge_args += [
            '/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ]
        bridge_remappings += [ ('/scan', '/laser/scan'),
                               ('/camera_info', '/depth_camera_info'), ]
        image_bridge_args = ['/depth_camera']
    else:
        image_bridge_args = ['/camera']

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_args,
        remappings=bridge_remappings,
    )

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

    position_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controllers", "-c", CM],
    )

    depthimage_to_laserscan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        remappings=[ ('/depth', '/depth_camera'), ('/scan', '/depth/scan'), ],
        parameters=[dtl_params_file]
    )

    pointcloud_to_laserscan_node = Node(
        name='pointcloud_to_laserscan',
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        parameters=[ptl_params_file]
    )

    ros2_laser_scan_merger_node = Node(
        package='ros2_laser_scan_merger',
        executable='ros2_laser_scan_merger',
        parameters=[rlsm_params_file],
        output='screen',
        respawn=True,
        respawn_delay=2
    )

    slam_toolbox_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('slam_toolbox'),
                        'launch'),
                        '/online_async_launch.py'
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file' : slam_params_file,
        }.items(),
    )

    static_transform_publisher_base_link = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments= ["--frame-id", "turbopi/base_link",
                        "--child-frame-id", "base_link"]
    )

    static_transform_publisher_camera = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments= ["--frame-id", "camera",
                        "--child-frame-id", "turbopi/base_link/camera"]
    )

    static_transform_publisher_depth_camera = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments= ["--frame-id", "depth_camera",
                        "--child-frame-id", "turbopi/base_link/depth_camera"]
    )

    static_transform_publisher_laser = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments= ["--frame-id", "chassis",
                        "--child-frame-id", "laser"]
    )

    static_transform_publisher_lidar = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments= ["--frame-id", "lidar",
                        "--child-frame-id", "turbopi/base_link/lidar_sensor"]
    )

    static_transform_publisher_sonar = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments= ["--frame-id", "sonar",
                        "--child-frame-id", "turbopi/base_link/sonar_sensor"]
    )

    delayed_static_transform_publisher_base_link = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=create_entity,
            on_exit=[static_transform_publisher_base_link],
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

    delayed_static_transform_publisher_laser = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=create_entity,
            on_exit=[static_transform_publisher_laser],
        )
    )

    delayed_static_transform_publisher_lidar = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=create_entity,
            on_exit=[static_transform_publisher_lidar],
        )
    )

    delayed_static_transform_publisher_sonar = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=create_entity,
            on_exit=[static_transform_publisher_sonar],
        )
    )

    delayed_depthimage_to_laserscan_node_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=create_entity,
            on_exit=[depthimage_to_laserscan_node],
        )
    )

    delayed_pointcloud_to_laserscan_node_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=create_entity,
            on_exit=[pointcloud_to_laserscan_node],
        )
    )

    delayed_ros2_laser_scan_merger_node_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=create_entity,
            on_exit=[ros2_laser_scan_merger_node],
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
            target_action=create_entity,
            on_exit=[position_spawner],
        )
    )

    nodes = [
        gazebo,
        create_entity,
        bridge,
        image_bridge,
        node_robot_state_publisher,
    ]

    # Enable features for custom robot style 3d depth camera or 2d camera
    if custom:
        nodes += [ 
            delayed_static_transform_publisher_depth_camera,
            delayed_static_transform_publisher_laser,
            delayed_depthimage_to_laserscan_node_spawner,
            delayed_pointcloud_to_laserscan_node_spawner,
            delayed_ros2_laser_scan_merger_node_spawner,
        ]
    else:
        nodes += [
            delayed_static_transform_publisher_camera,
            delayed_position_spawner,
        ]

    nodes += [
        delayed_static_transform_publisher_base_link,
        delayed_static_transform_publisher_lidar,
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
