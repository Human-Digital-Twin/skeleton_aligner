import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("skeleton_aligner")
    default_marker_config = os.path.join(pkg_share, "config", "marker_ids.yaml")

    marker_config_arg = DeclareLaunchArgument(
        "marker_config",
        default_value=default_marker_config,
        description="Path to the YAML file containing marker IDs configuration.",
    )

    weight_arg = DeclareLaunchArgument(
        "weight",
        default_value="1.0",
        description="Weight factor for cluster alignment.",
    )

    topic1_arg = DeclareLaunchArgument(
        "topic1",
        default_value="/hiros/kinect/skeleton",
        description="First input StampedSkeleton topic to align.",
    )

    topic2_arg = DeclareLaunchArgument(
        "topic2",
        default_value="/hiros/xsens/skeleton",
        description="Second input StampedSkeleton topic to align.",
    )

    skeleton_aligner_node = Node(
        package="skeleton_aligner",
        executable="skeleton_aligner",
        name="skeleton_aligner",
        output="screen",
        emulate_tty=True,
        parameters=[
            LaunchConfiguration("marker_config"),
            {
                "input_topics": [
                    LaunchConfiguration("topic1"),
                    LaunchConfiguration("topic2"),
                ]
            },
            {"weight": LaunchConfiguration("weight")},
        ],
    )

    return LaunchDescription(
        [
            marker_config_arg,
            weight_arg,
            topic1_arg,
            topic2_arg,
            skeleton_aligner_node,
        ]
    )
