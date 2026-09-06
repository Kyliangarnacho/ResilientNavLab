from setuptools import find_packages, setup
from glob import glob


package_name = 'resilient_nav_brne'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml', 'UPSTREAM.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/models', glob('models/*.sdf')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=[
        'setuptools',
        'numpy>=1.24,<2.3',
        'numba==0.61.2',
    ],
    zip_safe=True,
    maintainer='Kylian',
    maintainer_email='kylian@example.com',
    description='Pinned BRNE wrapper, LiDAR dynamic-agent input, and armed Gazebo demos.',
    license='GPL-3.0-only',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'brne_pedestrian_response_probe = resilient_nav_brne.brne_pedestrian_response_probe:main',
            'brne_pedestrian_odometry_adapter = resilient_nav_brne.brne_pedestrian_odometry_adapter:main',
            'brne_lidar_dynamic_agent_node = resilient_nav_brne.brne_lidar_dynamic_agent_node:main',
            'brne_pedestrian_stability_probe = resilient_nav_brne.brne_pedestrian_stability_probe:main',
            'brne_periodic_planner = resilient_nav_brne.brne_periodic_planner:main',
            'brne_crossing_pedestrian_driver = resilient_nav_brne.brne_crossing_pedestrian_driver:main',
            'brne_scene2_coordinator = resilient_nav_brne.brne_scene2_coordinator:main',
            'rpp_scene1_goal_coordinator = resilient_nav_brne.rpp_scene1_goal_coordinator:main',
            'brne_control_gate = resilient_nav_brne.brne_control_gate:main',
            'brne_task1_closeout_observer = resilient_nav_brne.brne_task1_closeout_observer:main',
            'brne_shadow_node = resilient_nav_brne.brne_shadow_node:main',
            'brne_shadow_smoke = resilient_nav_brne.brne_shadow_smoke:main',
            'brne_shadow_input_adapter = resilient_nav_brne.brne_shadow_input_adapter:main',
            'brne_synthetic_pedestrian_source = resilient_nav_brne.brne_synthetic_pedestrian_source:main',
        ],
    },
)
