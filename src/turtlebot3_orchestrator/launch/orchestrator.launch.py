from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch file for turtlebot3 pilot and orchestrator.
    
    This launch file starts both:
    - turtlebot3_pilot (lifecycle node)
    - turtlebot3_orchestrator (manages the pilot based on gestures)
    """
    return LaunchDescription([
        # Launch the pilot node
        Node(
            package='turtlebot3_pilot',
            executable='pilot',
            name='turtlebot3_pilot',
            output='screen',
            parameters=[],
        ),
        
        # Launch the orchestrator node
        Node(
            package='turtlebot3_orchestrator',
            executable='orchestrator',
            name='turtlebot3_orchestrator',
            output='screen',
            parameters=[],
        ),
    ])
