"""Top-level launch: sim + perception + behavior, all together."""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_share = get_package_share_directory("tb3_follower_bringup")

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "sim.launch.py"))
    )
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "perception.launch.py"))
    )
    behavior = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "behavior.launch.py"))
    )

    # Delay perception + behavior until Gazebo is up and camera/scan topics are publishing.
    return LaunchDescription([
        sim,
        TimerAction(period=8.0, actions=[perception]),
        TimerAction(period=9.0, actions=[behavior]),
    ])
