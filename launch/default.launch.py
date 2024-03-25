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
                {'kinect_marker_ids.pelvis': 0},
                {'kinect_marker_ids.right_hip': 22},
                {'kinect_marker_ids.left_hip': 18},
                {'xsens_marker_ids.pelvis': 1},
                {'xsens_marker_ids.right_hip': 16},
                {'xsens_marker_ids.left_hip': 20},
                {'kinect_input_topic': '/kinect/input/skeleton'},
                {'xsens_input_topic': '/xsens/input/skeleton'},
                {'output_topic': '/fused/output/skeleton'},
                {'publish_tfs': False},
            ]
        )
    ])
