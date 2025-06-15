from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import  ExecuteProcess
import os


def generate_launch_description():
    package_name = 'sage_commander'
    pkg_share = FindPackageShare(package_name).find(package_name)
    rviz_config = os.path.join(pkg_share, 'rviz', 'explorer_ws.rviz')

    semantic_frontier_exploration_share = FindPackageShare('semantic_frontier_exploration').find('semantic_frontier_exploration')
    semantic_frontier_exploration_param_file = os.path.join(semantic_frontier_exploration_share, 'config', 'sem_frontiers.yml')

    semantic_frontiers_node = Node(
        package='semantic_frontier_exploration',
        executable='semantic_frontier_exploration_node',
        name="semantic_frontier_exploration_node",
        output='screen',
        emulate_tty=True,
        parameters=[semantic_frontier_exploration_param_file]
    )

    rviz_node = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        semantic_frontiers_node,
        rviz_node
    ])
