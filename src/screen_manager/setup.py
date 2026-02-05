from setuptools import find_packages, setup

package_name = 'screen_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jilian Lubrat',
    maintainer_email='jilian.lubrat@etu.univ-cotedazur.fr',
    description='Gazebo screen management with distance-based safety.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'manage_screens = screen_manager.node:main',
        ],
    },
)