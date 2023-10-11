from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('device_name', default_value='/dev/ttyACM0', description='USB device name'),
        DeclareLaunchArgument('baud_rate', default_value='115200', description='Serial baud rate'),
        DeclareLaunchArgument('flow_control', default_value='none', description='Flow control (none/hardware/software)'),
        DeclareLaunchArgument('parity', default_value='none', description='Parity (none/odd/even)'),
        DeclareLaunchArgument('stop_bits', default_value="1", description='Stop bits (1/1.5/2)'),

        Node(
            package='robotont_driver',
            namespace='robotont_driver',
            executable='driver_node',
            name='driver',
            parameters=[{'device_name': LaunchConfiguration('device_name'), 
                         'baud_rate': LaunchConfiguration('baud_rate'),
                         'flow_control': LaunchConfiguration('flow_control'),
                         'parity': LaunchConfiguration('parity'),
                         'stop_bits': LaunchConfiguration('stop_bits')
                        }],
        ),
    ])