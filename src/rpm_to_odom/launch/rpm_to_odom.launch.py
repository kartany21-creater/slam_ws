from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rpm_to_odom',
            executable='rpm_to_odom_node',
            name='rpm_to_odom_node',
            output='screen',
        ),
    ])

