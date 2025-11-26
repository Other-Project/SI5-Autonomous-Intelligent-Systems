import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration("params_file")
    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    name = LaunchConfiguration('name').perform(context)
    
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(depthai_prefix, 'launch', 'example_segmentation.launch.py')),
            launch_arguments={"name": name,
                              "params_file": params_file}.items()),

        LoadComposableNodes(
            target_container=name+"_container",
            composable_node_descriptions=[
                    ComposableNode(
                        package="depthai_filters",
                        plugin="depthai_filters::SegmentationOverlay",
                        remappings=[('rgb/preview/image_raw', name+'/nn/passthrough/image_raw'),
                                    ('nn/image_raw', name+'/nn/image_raw')]
                    ),
            ],
        ),

    ]


def generate_launch_description():
    package = get_package_share_directory("camera_reader")

    declared_arguments = [
        DeclareLaunchArgument("name", default_value="camera"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(package, 'config', 'segmentation.yml')),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
