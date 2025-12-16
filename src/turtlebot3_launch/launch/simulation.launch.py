#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    IncludeLaunchDescription,
)
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_file_dir = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"), "launch"
    )
    ros_gz_sim = get_package_share_directory("ros_gz_sim")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    x_pose = LaunchConfiguration("x_pose", default="-2.0")
    y_pose = LaunchConfiguration("y_pose", default="-0.5")

    world = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"),
        "worlds",
        "turtlebot3_world.world",
    )

    box_sdf_path = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"),
        "models",
        "custom_box",
        "model.sdf",
    )

    ld = LaunchDescription()

    ld.add_action(
        AppendEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            os.path.join(
                get_package_share_directory("turtlebot3_descriptions"), "meshes"
            ),
        )
    )
    ld.add_action(AppendEnvironmentVariable("TURTLEBOT3_MODEL", "burger"))

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")
            ),
            launch_arguments={"gz_args": ["-r -s ", world]}.items(),
        )
    )
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")
            ),
            launch_arguments={"gz_args": "-g "}.items(),
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, "spawn_turtlebot3.launch.py")
            ),
            launch_arguments={"x_pose": x_pose, "y_pose": y_pose}.items(),
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("turtlebot3_launch"), "launch", "common.launch.py"
                )
            ),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
        )
    )

    ld.add_action(
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'my_custom_box',
                '-file', box_sdf_path,
                '-x', '1.0',
                '-y', '-0.5',
                '-z', '1.0'
            ],
            output='screen',
        )
    )

    return ld
