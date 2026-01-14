#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import    IncludeLaunchDescription


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("turtlebot3_launch"), "launch", "common_computer.launch.py"
                )
            ),
            launch_arguments={"use_sim_time": "false"}.items(),
        )
    )

    return ld
