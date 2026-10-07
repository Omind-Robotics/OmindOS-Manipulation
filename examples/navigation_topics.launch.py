"""Launch the published Navigation v0.1 nodes without its goal-overwriting supervisor."""
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='nav3d_native', executable='simulator', parameters=[{'mapping_phase_seconds': 5.0}]),
        Node(package='nav3d_native', executable='mapper', parameters=[{'map_output': '/tmp/omindos_v02_map'}]),
        TimerAction(period=6.0, actions=[Node(package='nav3d_native', executable=name)
                                       for name in ('localizer', 'planner', 'controller')])
    ])
