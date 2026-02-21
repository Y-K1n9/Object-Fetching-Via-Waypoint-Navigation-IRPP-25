from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'object_fetcher'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # Install config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.rviz')),
        # Install world files
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world')),
        # Install map files
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team IRPP-25',
    maintainer_email='student@university.edu',
    description='Autonomous object fetching via waypoint navigation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Main mission orchestrator (the "brain")
            'main_controller = object_fetcher.main_controller:main',
            # Sends navigation goals to Nav2
            'waypoint_navigator = object_fetcher.waypoint_navigator:main',
            # Detects ArUco markers with camera
            'marker_detector = object_fetcher.marker_detector:main',
            # Picks which zone to visit next
            'task_scheduler = object_fetcher.task_scheduler:main',
            # Mock pickup site confirmation node
            'pickup_site_node = object_fetcher.pickup_site_node:main',
        ],
    },
)
