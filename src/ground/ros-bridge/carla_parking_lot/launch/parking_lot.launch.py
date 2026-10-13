from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    host_arg = DeclareLaunchArgument(
        'host',
        default_value='192.168.43.1',
        description='CARLA 服务器 IP 地址'
    )

    parking_lot_node = Node(
        package='carla_parking_lot',
        executable='main',
        name='carla_parking_lot',
        output='screen',
        parameters=[{'host': LaunchConfiguration('host')}]
    )

    return LaunchDescription([
        host_arg,
        parking_lot_node,
    ])
