from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    launch_gazebo_arg = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('ros_ign_gazebo'), '/launch', '/ign_gazebo.launch.py']),
        launch_arguments={
#            'ign_args' : os.path.join(pkg_path,'worlds','world.sdf')
        }.items(),
    )

    node_spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=["-topic", "robot_description"]
    )

    return LaunchDescription([
        launch_gazebo_arg,
        node_spawn_robot,
    ])
