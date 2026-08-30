from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'resilient_nav_navigation'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    py_modules=[
        'amcl_evaluator',
        'costmap_contract',
        'costmap_experiment_evaluator',
        'costmap_joint_probe',
        'global_costmap_probe',
        'initial_pose_helper',
        'local_costmap_probe',
        'localization_probe',
        'localization_lifecycle_probe',
        'map_server_probe',
        'planner_probe',
        'controller_probe',
        'navigate_to_pose_probe',
        'navigation_benchmark_metrics',
        'navigation_benchmark_runner',
        'navigation_benchmark_gt_recorder',
        'navigation_benchmark_evaluator',
        'navigation_benchmark_batch',
        'navigation_obstacle_event_injector',
        'navigation_robustness_metrics',
        'navigation_robustness_trial',
        'path_safety',
        'phase9_assets',
    ],
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
        (
            os.path.join('share', package_name, 'models', 'phase10_task5_unmapped_box'),
            glob(os.path.join('models', 'phase10_task5_unmapped_box', '*')),
        ),
        (
            os.path.join('share', package_name, 'models', 'phase10_task5_blocking_wall'),
            glob(os.path.join('models', 'phase10_task5_blocking_wall', '*')),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kylian',
    maintainer_email='kylian@example.com',
    description='Healthy Nav2 localization integration resources.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'phase10_map_server_probe = map_server_probe:main',
            'phase10_initial_pose_helper = initial_pose_helper:main',
            'phase10_localization_probe = localization_probe:main',
            'phase10_localization_lifecycle_probe = localization_lifecycle_probe:main',
            'phase10_amcl_evaluator = amcl_evaluator:main',
            'phase10_global_costmap_probe = global_costmap_probe:main',
            'phase10_local_costmap_probe = local_costmap_probe:main',
            'phase10_costmap_joint_probe = costmap_joint_probe:main',
            'phase10_costmap_experiment_evaluator = costmap_experiment_evaluator:main',
            'phase10_planner_probe = planner_probe:main',
            'phase10_controller_probe = controller_probe:main',
            'phase10_navigate_to_pose_probe = navigate_to_pose_probe:main',
            'phase10_navigation_benchmark_runner = navigation_benchmark_runner:main',
            'phase10_navigation_benchmark_gt_recorder = navigation_benchmark_gt_recorder:main',
            'phase10_navigation_benchmark_evaluator = navigation_benchmark_evaluator:main',
            'phase10_navigation_benchmark_batch = navigation_benchmark_batch:main',
            'phase10_navigation_obstacle_event_injector = navigation_obstacle_event_injector:main',
            'phase10_navigation_robustness_trial = navigation_robustness_trial:main',
        ],
    },
)
