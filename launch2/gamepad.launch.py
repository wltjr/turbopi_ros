
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

def generate_launch_description():

    spawn_joy_linux_node = Node(package='joy_linux',
                                executable='joy_linux_node',
                                name='joy_linux_node',
                                parameters=[])

    spawn_teleop_turbopi_node = Node(package='turbopi_ros',
                                        executable='teleop_turbopi_node',
                                        name='teleop_turbopi_node',
                                        parameters=[])

    delayed_teleop_turbopi_node_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=spawn_joy_linux_node,
            on_start=[spawn_teleop_turbopi_node],
        )
    )

    return LaunchDescription([
        spawn_joy_linux_node,
        delayed_teleop_turbopi_node_spawner,
    ])
