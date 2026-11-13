"""Bring up the Bayani-6 nodes. Start the LIMO driver, camera, Cartographer and Nav2 first."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("bayani_core")
    return LaunchDescription([
        DeclareLaunchArgument("waypoints", default_value=os.path.join(share, "config", "patrol_waypoints.yaml")),
        DeclareLaunchArgument("threshold", default_value="0.8"),
        DeclareLaunchArgument("calibration_video", default_value=""),
        DeclareLaunchArgument("arm_port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("arm_dry_run", default_value="false"),
        Node(
            package="bayani_core", executable="anomaly_detector", output="screen",
            parameters=[{"calibration_video": LaunchConfiguration("calibration_video")}],
        ),
        Node(
            package="bayani_core", executable="arm_sampler", output="screen",
            parameters=[{"port": LaunchConfiguration("arm_port"), "dry_run": LaunchConfiguration("arm_dry_run")}],
        ),
        Node(
            package="bayani_core", executable="mission_master", output="screen",
            parameters=[{"waypoints_file": LaunchConfiguration("waypoints"), "threshold": LaunchConfiguration("threshold")}],
        ),
    ])
