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
                {'kinect_input_topic': '/kinect/input/skeleton'},
                {'xsens_input_topic': '/xsens/input/skeleton'},
                {'output_topic': '/aligned/output/skeleton'},
                {'publish_tfs': False},
                marker_ids_cfg,
            ]
        )
    ])
