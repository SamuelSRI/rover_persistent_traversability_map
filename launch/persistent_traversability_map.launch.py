from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_share = get_package_share_directory("rover_persistent_traversability_map")

    config_file = os.path.join(
        pkg_share,
        "config",
        "persistent_traversability_map.yaml"
    )

    return LaunchDescription([
        Node(
            package="rover_persistent_traversability_map",
            executable="persistent_traversability_map_node",
            name="persistent_traversability_map_node",
            output="screen",
            parameters=[config_file],
        )
    ])
