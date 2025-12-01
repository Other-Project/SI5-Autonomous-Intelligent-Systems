import os
from launch import LaunchDescription
# L'import de Node est essentiel pour lancer un exécutable
from launch_ros.actions import Node 


def generate_launch_description():
    
    return LaunchDescription([
        
        # 1. LANCER LE SCRIPT READER.PY EN TANT QUE NŒUD ROS 2
        Node(
            package="camera_reader",  # Votre package ROS 2
            executable="reader_node", # L'exécutable déclaré dans setup.py
            name="camera_reader_node", # Le nom que ce noeud aura dans ros2 node list
            output="screen",
            # Vous pouvez ajouter ici des arguments (args) ou des paramètres (parameters) 
            # si votre script reader.py en a besoin.
        ),
    ])