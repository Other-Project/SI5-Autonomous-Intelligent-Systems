# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ["TURTLEBOT3_MODEL"]

    # Launch configuration variables specific to simulation
    x_pose = LaunchConfiguration("x_pose", default="0.0")
    y_pose = LaunchConfiguration("y_pose", default="0.0")

    # Get the urdf file
    urdf_path = os.path.join(
        get_package_share_directory("turtlebot3_descriptions"),
        "urdf",
        "turtlebot3_burger_oak_d_pro.urdf",
    )
    robot_description_config = xacro.process_file(urdf_path).toxml()

    bridge_config = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"),
        "params",
        "turtlebot3_bridge.yaml",
    )

    ld = LaunchDescription()

    # Declare the launch arguments
    ld.add_action(
        DeclareLaunchArgument(
            "x_pose", default_value="0.0", description="X position of the robot"
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "y_pose", default_value="0.0", description="Y position of the robot"
        )
    )

    ld.add_action(
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-name",
                TURTLEBOT3_MODEL,
                "-string",
                robot_description_config,
                "-x",
                x_pose,
                "-y",
                y_pose,
                "-z",
                "0.01",
            ],
            output="screen",
        )
    )

    ld.add_action(
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "--ros-args",
                "-p",
                f"config_file:={bridge_config}",
            ],
            output="screen",
        )
    )

    return ld
