from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yorizori',
            executable='command_publisher_node',
            name='command_publisher',
            output='screen',
        ),
        Node(
            package='yorizori',
            executable='robot_control_node',
            name='robot_control',
            output='screen',
        ),
        Node(
            package='yorizori',
            executable='suction_cup_node',  # 새로운 흡입컵 노드 추가
            name='suction_cup_control',
            output='screen',
        ),
    ])
