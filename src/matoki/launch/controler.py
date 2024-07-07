from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the joy node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'dev': '/dev/input/js0',  # Adjust the device if necessary
                'deadzone': 0.1,
                'autorepeat_rate': 20.0,
                'coalesce_interval': 0.001,
            }]
        ),

        # Launch the RC override node
        Node(
            package='matoki',
            executable='rc_in',
            name='rc_override_node',
            output='screen'
        ),
    ])

