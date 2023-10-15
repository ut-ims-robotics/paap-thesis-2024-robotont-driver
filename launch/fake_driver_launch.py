from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robotont_driver',
            namespace='robotont_driver',
            executable='fake_driver_node',
            name='driver',
            parameters=[],
        ),
    ])