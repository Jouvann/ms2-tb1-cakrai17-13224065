from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pkg_13224065',
            executable='node_hubung',
            name='hubung_node',
            parameters=['config/hubung_params.yaml'],
            output='screen'
        )
    ])

