from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import UnlessCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    robot_description = ParameterValue(
        Command(['xacro ', str(get_package_share_path('akros2_description') / 'urdf/robot.urdf.xacro')]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(
            name='js_ext',
            default_value='True',
            description='Enable Joint States from external nodes (like the micro-ROS node). If False, enable Joint States from the Joint State Publisher'),

        DeclareLaunchArgument(
            name='js_topic',
            default_value='joint_states',
            description='Switch between measured (joint_states) and required (req_states) topics'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
            remappings=[
                ('/joint_states', ['/', LaunchConfiguration('js_topic')])
            ]),

        Node(
            condition=UnlessCondition(LaunchConfiguration('js_ext')),
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'source_list': ['joint_lf', 'joint_lb', 'joint_rf', 'joint_rb']}],
            remappings=[
                ('/joint_states', ['/', LaunchConfiguration('js_topic')])
            ]),
    ])
