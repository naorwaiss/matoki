from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            namespace='controller_1',
            name='joy_node_1',
            parameters=[{'dev': '/dev/input/js0'}, {'deadzone': 0.05}],
            output='screen'
        ),
        Node(
            package='joy',
            executable='joy_node',
            namespace='controller_2',
            name='joy_node_2',
            parameters=[{'dev': '/dev/input/js1'}, {'deadzone': 0.05}],
            output='screen'
        )
    ])

