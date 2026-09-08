from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'resilient_nav_fusion'


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
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py')),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))
            + glob(os.path.join('config', '*.rviz')),
        ),
        (
            os.path.join(
                'share', package_name, 'models', 'physical_reliability_rf_v2'
            ),
            glob(os.path.join('models', 'physical_reliability_rf_v2', '*.joblib'))
            + glob(os.path.join('models', 'physical_reliability_rf_v2', '*.json')),
        ),
    ],
    install_requires=['joblib', 'numpy', 'scikit-learn', 'setuptools'],
    zip_safe=True,
    maintainer='Kylian',
    maintainer_email='kylian@example.com',
    description=(
        'Pure-Python health-aware fusion policy contracts for '
        'ResilientNavLab.'
    ),
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            (
                'measurement_adapter = '
                'resilient_nav_fusion.measurement_adapter:main'
            ),
            (
                'healthy_smoke_probe = '
                'resilient_nav_fusion.healthy_smoke_probe:main'
            ),
            (
                'ground_truth_pose_adapter = '
                'resilient_nav_fusion.ground_truth_pose_adapter:main'
            ),
            (
                'localization_evaluator = '
                'resilient_nav_fusion.localization_evaluator_node:main'
            ),
            (
                'imu_bias_benchmark_runner = '
                'resilient_nav_fusion.imu_bias_benchmark_runner:main'
            ),
            (
                'imu_benchmark_runner = '
                'resilient_nav_fusion.imu_bias_benchmark_runner:main'
            ),
            (
                'trajectory_path_adapter = '
                'resilient_nav_fusion.trajectory_path_adapter:main'
            ),
            (
                'lidar_projection_evaluator = '
                'resilient_nav_fusion.lidar_projection_evaluator:main'
            ),
            (
                'lidar_odometry = '
                'resilient_nav_fusion.lidar_odometry:main'
            ),
            (
                'physical_reliability_dataset = '
                'resilient_nav_fusion.physical_reliability_dataset:main'
            ),
            (
                'physical_reliability_rf = '
                'resilient_nav_fusion.physical_reliability_rf:main'
            ),
        ],
    },
)
