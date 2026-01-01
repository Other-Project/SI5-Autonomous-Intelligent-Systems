from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'camera_reader'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'data'), glob('camera_reader/*.json')), 
        (os.path.join('share', package_name, 'models'), glob('camera_reader/models/*.blob')),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'opencv-python',
        'depthai'
    ],
    zip_safe=True,
    maintainer='evan',
    maintainer_email='evan.galli@etu.univ-cotedazur.fr',
    description='Camera reader node for ROS2',
    license='Apache 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'reader_node = camera_reader.reader:main', 
        ],
    },
)
