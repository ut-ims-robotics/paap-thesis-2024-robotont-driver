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
        DeclareLaunchArgument('stop_bits', default_value='one', description='Stop bits (one/one_point_five/two)'),
        DeclareLaunchArgument('plugin_odom', default_value='True', description='Odom plugin =active'),
        DeclareLaunchArgument('plugin_motor', default_value='True', description='Motors plugin =active'),
        DeclareLaunchArgument('plugin_led_module', default_value='True', description='LED plugin =active'),
        DeclareLaunchArgument('plugin_power_supply', default_value='False', description='Power supply plugin =active'),
        DeclareLaunchArgument('plugin_range', default_value='False', description='Range plugin =active'),

        Node(
            package='robotont_driver',
            namespace='robotont_driver',
            executable='driver_node',
            name='driver',
            parameters=[{'device_name': LaunchConfiguration('device_name'), 
                         'baud_rate': LaunchConfiguration('baud_rate'),
                         'flow_control': LaunchConfiguration('flow_control'),
                         'parity': LaunchConfiguration('parity'),
                         'stop_bits': LaunchConfiguration('stop_bits'),
                         'plugin_odom' : LaunchConfiguration('plugin_odom'),
                         'plugin_motor' : LaunchConfiguration('plugin_motor'),
                         'plugin_led_module' : LaunchConfiguration('plugin_led_module'),
                         'plugin_power_supply' : LaunchConfiguration('plugin_power_supply'),
                         'plugin_range' : LaunchConfiguration('plugin_range')
                        }],
        ),
    ])