from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='skeleton_fusion',
            executable='skeleton_fusion',
            name='skeleton_fusion',
            namespace='hiros',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'kinect_input_topic': '/kinect/input/skeleton'},
                {'xsens_input_topic': '/xsens/input/skeleton'},
                {'output_topic': '/fused/output/skeleton'},
                {'publish_tfs': False},
            ]
        )
    ])
