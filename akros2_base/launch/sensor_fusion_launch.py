from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    imu_filter_config_path = os.path.join(
        get_package_share_directory('akros2_base'),
        'config',
        'imu_filter_config.yaml')

    ekf_config_path = os.path.join(
        get_package_share_directory('akros2_base'),
        'config',
        'ekf_config.yaml')

    motion_detector_config_path = os.path.join(
        get_package_share_directory('akros2_base'),
        'config',
        'motion_detector_config.yaml')

    return LaunchDescription([
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            output='screen',
            parameters=[imu_filter_config_path],
            remappings=[
                ('/imu/data_raw', '/imu'),
                ('/imu/data', '/imu/filtered')]),

        Node(
            package='akros2_base',
            executable='motion_detector',
            name='motion_detector',
            output='screen',
            parameters=[motion_detector_config_path],
            remappings=[('/imu', '/imu/filtered')]),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[ekf_config_path]),
    ])
