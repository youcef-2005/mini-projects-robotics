from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mini_project_10',
            executable='dummy_node',
            name='node_alpha'
        ),
        Node(
            package='mini_project_10',
            executable='dummy_node',
            name='node_beta'
        )
    ])
