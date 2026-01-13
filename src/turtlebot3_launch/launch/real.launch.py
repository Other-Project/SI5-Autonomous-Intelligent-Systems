#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    rviz_config_path = os.path.join(
        get_package_share_directory("turtlebot3_descriptions"), "rviz", "model.rviz"
    )

    ld.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            parameters=[{"use_sim_time": False}],
            arguments=["-d", rviz_config_path],
        ),
    )

    return ld
