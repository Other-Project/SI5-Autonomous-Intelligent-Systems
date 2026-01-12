from launch import LaunchDescription
from launch_ros.actions import Node 

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="camera_reader_simulation",
            executable="reader_simulation_node",
            name="camera_reader_simulation_node",
            output="screen",
        ),
    ])
