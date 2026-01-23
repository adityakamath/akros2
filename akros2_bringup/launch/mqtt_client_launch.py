from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    mqtt_client_config_path = PathJoinSubstitution(
        [FindPackageShare("akros2_bringup"), "config", "mqtt_client_config.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument(
            name='broker_host',
            default_value='ak-win11',
            description='Hostname of the MQTT Broker'),

        DeclareLaunchArgument(
            name='broker_port',
            default_value=1883,
            description='Port used by the MQTT Broker'),

        Node(
            package='mqtt_client',
            executable='mqtt_client',
            name='mqtt_client_node',
            output='screen',
            parameters=[{'broker': {'host': LaunchConfiguration('broker_host'),
                                    'port': LaunchConfiguration('broker_port')}},
                        mqtt_client_config_path]),
    ])