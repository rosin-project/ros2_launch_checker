from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            node_name='stresser_' + str(i),
            package='tf_stresser',
            node_executable='stressing',
            output='screen',
            parameters=[{
                'frame_prefix': 'tree' + str(i),
            }]) for i in range(5)
    ])
