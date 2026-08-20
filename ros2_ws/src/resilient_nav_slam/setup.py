from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'resilient_nav_slam'


def map_data_files():
    """Preserve the maps/ subtree when installing package-share resources."""
    return [
        (
            os.path.join('share', package_name, directory),
            [os.path.join(directory, filename) for filename in filenames],
        )
        for directory, _, filenames in os.walk('maps')
        if filenames
    ]


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml')),
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py')),
        ),
        (
            os.path.join('share', package_name, 'rviz'),
            glob(os.path.join('rviz', '*.rviz')),
        ),
    ] + map_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kylian',
    maintainer_email='kylian@example.com',
    description='Healthy 2D Slam Toolbox integration resources.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'slam_probe = resilient_nav_slam.slam_probe:main',
            'slam_pose_adapter = resilient_nav_slam.slam_pose_adapter:main',
            'slam_map_adapter = resilient_nav_slam.slam_map_adapter:main',
            'slam_evaluator = resilient_nav_slam.slam_evaluator_node:main',
        ],
    },
)
