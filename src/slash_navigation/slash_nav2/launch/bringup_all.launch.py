#!/usr/bin/env python3
"""
Launch file to concurrently include multiple package launch files:
 - fast_lio/mapping.launch.py
 - linefit_ground_segmentation_ros/segmentation.launch.py
 - pointcloud_to_laserscan/pointcloud_to_laserscan_launch.py
 - slash_nav2/bringup_real.launch.py

Usage:
  ros2 launch slash_nav2 bringup_all.launch.py

This file simply includes the other launch files so they are launched together.
"""
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    launches = [
        ("fast_lio", "mapping.launch.py"),
        ("linefit_ground_segmentation_ros", "segmentation.launch.py"),
        ("pointcloud_to_laserscan", "pointcloud_to_laserscan_launch.py"),
        ("slash_nav2", "bringup_real.launch.py"),
        #("slash_nav2", "bringup_with_dll.launch.py"),
    ]

    for pkg, lf in launches:
        try:
            pkg_share = get_package_share_directory(pkg)
        except Exception as e:
            # If the package is not found at launch file parse time, include a log message.
            ld.add_action(LogInfo(msg=f"Package '{pkg}' not found: {e}"))
            continue

        launch_path = os.path.join(pkg_share, 'launch', lf)

        if not os.path.exists(launch_path):
            ld.add_action(LogInfo(msg=f"Launch file not found: {launch_path}"))
            continue


        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_path)
        ))

    return ld
