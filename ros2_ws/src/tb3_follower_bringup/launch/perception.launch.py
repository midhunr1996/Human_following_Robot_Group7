"""Launch the YOLOv8 person detector node with its params YAML."""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    perception_share = get_package_share_directory("tb3_follower_perception")
    params = os.path.join(perception_share, "config", "detector_params.yaml")

    return LaunchDescription([
        Node(
            package="tb3_follower_perception",
            executable="person_detector_node",
            name="person_detector_node",
            output="screen",
            parameters=[params],
            emulate_tty=True,
        ),
    ])
