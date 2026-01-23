from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    joy_launch_path = PathJoinSubstitution(
        [FindPackageShare('akros2_base'), 'launch', 'joy_launch.py'])

    viz_launch_path = PathJoinSubstitution(
        [FindPackageShare('akros2_bringup'), 'launch', 'viz_launch.py'])

    return LaunchDescription([
        DeclareLaunchArgument(
            name='joy_config',
            default_value='steamdeck',
            description='Select Controller: ps4 (PS4/DS4), stadia (Google Stadia), sn30pro (8BitDo SN30 Pro), steamdeck (Valve Steam Deck), none (Disabled)'),

        DeclareLaunchArgument(
            name='viz_config',
            default_value='none',
            description='Select Visualization Config: foxglove (Foxglove Bridge), rosbridge (ROSBridge Server), rviz (RViz Config), none (Disabled)'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(joy_launch_path),
            launch_arguments={'joy_config': LaunchConfiguration('joy_config')}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(viz_launch_path),
            launch_arguments={'viz_config': LaunchConfiguration('viz_config')}.items()),
    ])
