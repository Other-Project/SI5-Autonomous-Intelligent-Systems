from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot3_braitenberg',
            executable='vehicle2a_node',
            name='vehicle2a',
            output='screen',
        )
    ])

