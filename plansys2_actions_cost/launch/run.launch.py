# File: launch.py
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='plansys2_actions_cost',
            executable='cost_test',
            name='test_node',
            output='screen'
        )
    ])
