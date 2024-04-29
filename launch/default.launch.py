import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    marker_ids_cfg = os.path.join(
        get_package_share_directory('skeleton_aligner'),
        'config',
        'marker_ids.yaml'
    )

    return LaunchDescription([
        Node(
            package='skeleton_aligner',
            executable='skeleton_aligner',
            name='skeleton_aligner',
            namespace='hiros',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'input_topics': ['topic1/skeleton', 'topic2/skeleton']},
                {'weight': 1.},
                marker_ids_cfg,
            ]
        )
    ])
