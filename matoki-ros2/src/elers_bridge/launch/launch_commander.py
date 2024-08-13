from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            #output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml'],
            #output='screen'
        ),
        # Node(
        #     package='elers_bridge',
        #     executable='old_aplication',
        #     name='old_aplication',
        #     output='screen'
        # ),
        # Node(
        #     package='elers_bridge',
        #     executable='connect_bridge',
        #     name='connect_bridge',
        #     output='screen'
        # )
    ])
