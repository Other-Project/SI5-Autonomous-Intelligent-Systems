#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch_ros.actions import Node, SetRemap
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_file_dir = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"), "launch"
    )

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    rviz_config_path = os.path.join(
        get_package_share_directory("turtlebot3_descriptions"), "rviz", "model.rviz"
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
                os.path.join(launch_file_dir, "robot_state_publisher.launch.py")
            ),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
        )
    )

    ld.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            arguments=["-d", rviz_config_path],
        ),
    )

    ld.add_action(
        GroupAction(
            actions=[
                SetRemap(src="/cmd_vel_nav", dst="/cmd_vel"),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory("nav2_bringup"),
                            "launch",
                            "navigation_launch.py",
                        )
                    ),
                    launch_arguments={
                        "use_sim_time": use_sim_time,
                        "params_file": os.path.join(
                            get_package_share_directory("turtlebot3_launch"),
                            "config",
                            "nav2_params.yaml",
                        ),
                        "log_level": "warn",
                    }.items(),
                ),
            ]
        )
    )

    ld.add_action(
        GroupAction(
            actions=[
                SetEnvironmentVariable('RCUTILS_LOGGING_MIN_SEVERITY', 'WARN'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory("slam_toolbox"),
                            "launch",
                            "online_async_launch.py",
                        )
                    ),
                    launch_arguments={"use_sim_time": use_sim_time}.items(),
                )
            ]
        )
    )

    ld.add_action(
        Node(
            package="turtlebot3_pilot",
            executable="pilot",
            name="turtlebot3_pilot",
            output="screen",
        )
    )

    ld.add_action(
        Node(
            package="turtlebot3_orchestrator",
            executable="orchestrator",
            name="turtlebot3_orchestrator",
            output="screen",
        )
    )

    return ld
