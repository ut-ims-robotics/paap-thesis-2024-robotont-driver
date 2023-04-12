from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robotont_driver',
            namespace='robotont_driver',
            executable='driver_node',
            name='driver'
        ),
    ])