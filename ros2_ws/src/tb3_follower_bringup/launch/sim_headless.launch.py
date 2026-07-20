"""Headless Gazebo (gzserver only, no gzclient) + TB3 spawn.

Same as sim.launch.py but without the 3D viewer, for low-RAM hosts. The follow
pipeline runs identically (camera/scan come from gzserver plugins); you just
don't get the interactive Gazebo window. Verify via /person/detection + /cmd_vel.
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_share = get_package_share_directory("tb3_follower_bringup")
    tb3_gazebo_share = get_package_share_directory("turtlebot3_gazebo")
    gazebo_ros_share = get_package_share_directory("gazebo_ros")

    world_path = os.path.join(bringup_share, "worlds", "follow_world.world")

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": world_path, "verbose": "false"}.items(),
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_share, "launch", "robot_state_publisher.launch.py")
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    spawn_tb3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_share, "launch", "spawn_turtlebot3.launch.py")
        ),
        launch_arguments={
            "x_pose": LaunchConfiguration("x_pose"),
            "y_pose": LaunchConfiguration("y_pose"),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("x_pose", default_value="0.0"),
        DeclareLaunchArgument("y_pose", default_value="0.0"),
        gzserver,
        robot_state_publisher,
        spawn_tb3,
    ])
