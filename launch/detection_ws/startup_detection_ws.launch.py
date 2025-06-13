from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from launch.actions import TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart


def generate_launch_description():
    seem_namespace = 'seem_ros'
    seem_name = "seem_lifecycle_node"
    # Lifecycle-Node für SEEM
    seem_node = LifecycleNode(
        package='seem_ros',
        executable='seem_lifecycle_node',
        name=seem_name,
        namespace=seem_namespace,
        output='screen',
    )

    # Steuerknoten für Lifecycle-Transitionen
    controller_node = Node(
        package='sage_commander',
        executable='lifecycle_controller',
        name='lifecycle_controller',
        output='screen',
        parameters=[{'target_node': f'/{seem_namespace}/{seem_name}',
                      'timeout_sec': 60.0
                     }],
    )

    # EventHandler: starte Controller, nachdem SEEM Node läuft (mit kurzem Delay)
    start_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=seem_node,
            on_start=[TimerAction(period=0.5, actions=[controller_node])]
        )
    )

    return LaunchDescription([
        seem_node,
        start_controller
    ])
