from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node

def generate_launch_description():
 
    seem_lifecycle_node = Node(
        package='seem_ros',
        executable='seem_lifecycle_node',
        name='seem_lifecycle_node',
        output='screen',
        namespace='seem_ros',
        parameters=[{
            'timer_frequency': 10.0,
            'config_name': 'focall_unicl_lang_demo.yaml'
        }]
    )

    lifecycle_controller = Node(
        package='sage_commander',
        executable='seem_lifecycle_controller',
        name='lifecycle_controller',
        output='screen'
    )


    return LaunchDescription([
        seem_lifecycle_node,
        lifecycle_controller
    ])
