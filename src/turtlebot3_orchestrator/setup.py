from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtlebot3_orchestrator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Antoine-Marie Michelozzi',
    maintainer_email='antoine-marie.michelozzi@etu.univ-cotedazur.fr',
    description='TurtleBot3 Orchestrator - Manages pilot lifecycle based on gestures',
    license='MIT',
    entry_points={
        'console_scripts': [
            'orchestrator = turtlebot3_orchestrator.orchestrator:main'
        ],
    },
)
