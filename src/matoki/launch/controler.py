from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess


def generate_launch_description():
    # Declare configuration arguments
    fcu_url = DeclareLaunchArgument(
        'fcu_url', default_value='serial:///dev/ttyUSB0',
        description='FCU URL for MAVROS'
    )

    #mabey some day i like to use the gcs for somthing
    # gcs_url = DeclareLaunchArgument(
    #     'gcs_url', default_value='udp://@127.0.0.1',
    #     description='GCS URL for MAVROS'
    # )

    # Start MAVROS node using ExecuteProcess with dynamic command generation
    mavros_process = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'mavros', 'mavros_node', '--ros-args',
            '-p', 'fcu_url:=serial:///dev/ttyUSB0',
            '-p', 'gcs_url:=udp://@127.0.0.1'
        ],
        output='screen'
    )







    return LaunchDescription([
        # Launch the joy node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            #output='screen',
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
            #output='screen'
        ),
    ])

