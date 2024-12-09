from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="kinematics",
            executable="forward_kinematics_node",
            name="forward_kinematics_node",
            output="screen",
            parameters=[
                # Add any parameters you want to pass to the node, e.g.,
                # {"parameter_name": value}
            ]
        )
    ])
