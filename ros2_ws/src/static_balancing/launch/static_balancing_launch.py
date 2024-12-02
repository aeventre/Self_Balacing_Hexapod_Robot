from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='static_balancing',
            executable='static_balancing_node',
            name='static_balancing_node',
            parameters=['config/params.yaml']
        )
    ])
