from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # YDLIDAR ノード
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            parameters=['/home/maffin21/ydlidar_ws/install/ydlidar_ros2_driver/share/ydlidar_ros2_driver/params/ydlidar.yaml'],
        ),

        # base_link → laser_frame の固定TF
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_laser',
            arguments=['0', '0', '0.02', '0', '0', '0', '1', 'base_link', 'laser_frame'],
        ),

        # rpm_to_odom ノード（/rpm → /odom + TF）
        Node(
            package='rpm_to_odom',
            executable='rpm_to_odom_node',
            name='rpm_to_odom_node',
            output='screen',
        ),

        # SLAM Toolbox のライフサイクルノード
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=['/home/maffin21/slam_ws/install/ydlidar_slam_launch/share/ydlidar_slam_launch/config/mapper_params_online_async.yaml'],
        ),

        # slam_toolbox を3秒後に configure
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
                    output='screen'
                )
            ]
        ),

        # slam_toolbox を6秒後に activate
        TimerAction(
            period=6.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
                    output='screen'
                )
            ]
        ),
    ])

