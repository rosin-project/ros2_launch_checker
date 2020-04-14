from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            node_name='stresser_a',
            package='tf_stresser',
            node_executable='stresser',
            output='screen',
            parameters=[{
                'frequency': 1000,
                'frame_prefix': 'a',
            }],
        ),
    ])
