from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gpio_sensors',
            executable='gpio_sensor_node',
            name='gpio_sensor_node',
            output='screen',
        ),
    ])