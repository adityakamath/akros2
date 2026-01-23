from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='akros2_teleop',
            executable='twist_mixer',
            name='twist_mixer',
            output='screen',
            parameters=[{'timer_period': 0.02}],
            remappings=[
                ('/teleop_vel', '/joy_vel'),
                ('/auto_vel', '/nav_vel'),
                ('/mix_vel', '/cmd_vel'),
            ]),
    ])
