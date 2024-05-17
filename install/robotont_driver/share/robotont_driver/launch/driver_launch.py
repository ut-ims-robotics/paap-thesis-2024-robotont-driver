from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    parameters_file_path = PathJoinSubstitution([
        FindPackageShare('robotont_driver'), 'config', 'parameters.yaml'
    ])

    print(f"Parameters file path: {parameters_file_path}")

    return LaunchDescription([

        LogInfo(msg=f"Using parameters file: {parameters_file_path}"),

        DeclareLaunchArgument('device_name', default_value='/dev/ttyACM5', description='USB device name'),
        DeclareLaunchArgument('baud_rate', default_value='115200', description='Serial baud rate'),
        DeclareLaunchArgument('flow_control', default_value='none', description='Flow control (none/hardware/software)'),
        DeclareLaunchArgument('parity', default_value='none', description='Parity (none/odd/even)'),
        DeclareLaunchArgument('stop_bits', default_value='one', description='Stop bits (one/one_point_five/two)'),
        DeclareLaunchArgument('plugin_odom', default_value='True', description='Odom plugin active'),
        DeclareLaunchArgument('plugin_motor', default_value='True', description='Motors plugin active'),
        DeclareLaunchArgument('plugin_led_module', default_value='True', description='LED plugin active'),
        DeclareLaunchArgument('plugin_power_supply', default_value='False', description='Power supply plugin active'),
        DeclareLaunchArgument('plugin_range', default_value='False', description='Range plugin active'),

        Node(
            package='robotont_driver',
            namespace='',
            executable='driver_node',
            name='driver',
            parameters=[parameters_file_path],
            output='screen',
            arguments=['--ros-args', '--log-level', 'info']

        ),
    ])

