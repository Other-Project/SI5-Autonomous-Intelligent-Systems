from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot3_braitenberg',
            executable='vehicle2b_node',
            name='vehicle2b',
            output='screen',
        )
    ])

