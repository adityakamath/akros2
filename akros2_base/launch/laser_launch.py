from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    laser_filter_config_path = PathJoinSubstitution(
        [FindPackageShare("akros2_base"), "config", "laser_filter_config.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument(
            name='laser_filter',
            default_value='True',
            description='Enable LIDAR Filter Chain'),

        Node(
            package='ldlidar',
            executable='ldlidar',
            name='laser_node',
            output='screen',
            parameters=[{'serial_port': '/dev/ttyLIDAR'},
                        {'topic_name': 'scan'},
                        {'lidar_frame': 'laser_frame'},
                        {'range_threshold': 0.005}
            ]),

        Node(
            condition=IfCondition(LaunchConfiguration('laser_filter')),
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='laser_filter_node',
            parameters=[laser_filter_config_path]),
    ])