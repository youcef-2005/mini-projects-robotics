from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction

def generate_launch_description():
    # Configuration pour Robot 1
    robot1_group = GroupAction(actions=[
        PushRosNamespace('robot1'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', name='rsp'),
        Node(package='nav2_bringup', executable='bringup_launch', name='nav2')
    ])

    # Configuration pour Robot 2
    robot2_group = GroupAction(actions=[
        PushRosNamespace('robot2'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', name='rsp'),
        Node(package='nav2_bringup', executable='bringup_launch', name='nav2')
    ])

    return LaunchDescription([
        robot1_group,
        robot2_group
    ])
