"""Launch the follower behavior tree node."""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    behavior_share = get_package_share_directory("tb3_follower_behavior")
    params = os.path.join(behavior_share, "config", "follower_params.yaml")

    return LaunchDescription([
        Node(
            package="tb3_follower_behavior",
            executable="follower_bt_node",
            name="follower_bt_node",
            output="screen",
            parameters=[params],
            emulate_tty=True,
        ),
    ])
