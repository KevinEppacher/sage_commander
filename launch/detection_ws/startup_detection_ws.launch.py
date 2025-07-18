from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from launch.actions import TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    package_name = 'sage_commander'
    pkg_share = FindPackageShare(package_name).find(package_name)
    param_file = os.path.join(pkg_share, 'config', 'startup_lifecycle_nodes.yaml')

    seem_name = "seem_lifecycle_node"
    seem_namespace = 'seem_ros'
    seem_node = Node(
        package='seem_ros',
        executable='seem_lifecycle_node',
        name=seem_name,
        namespace=seem_namespace,
        output='screen',
        emulate_tty=True
    )

    name = "value_map_node"
    value_map_namespace = 'value_map'
    value_map_node = Node(
        package="value_map",
        executable='value_map_node',
        name=name,
        namespace=value_map_namespace,
        output='screen',
        emulate_tty=True,
        # arguments=['--ros-args', '--log-level', 'debug']
    )

    controller_node = Node(
        package='sage_commander',
        executable='sequential_lifecycle_launch',
        name='sequential_lifecycle_launch',
        output='screen',
        emulate_tty=True,
        parameters=[param_file]
    )

    start_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=seem_node,
            on_start=[TimerAction(period=0.5, actions=[controller_node])]
        )
    )

    return LaunchDescription([
        seem_node,
        value_map_node,
        start_controller
    ])
