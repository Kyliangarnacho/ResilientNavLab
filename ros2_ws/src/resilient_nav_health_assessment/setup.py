from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'resilient_nav_health_assessment'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['numpy', 'setuptools'],
    zip_safe=True,
    maintainer='Kylian',
    maintainer_email='296032282@qq.com',
    description='Minimal sensor health assessment publishers for ResilientNavLab.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            (
                'sensor_health_monitor = '
                'resilient_nav_health_assessment.sensor_health_monitor:main'
            ),
            (
                'health_evaluator = '
                'resilient_nav_health_assessment.health_evaluator:main'
            ),
            (
                'camera_health_feature_demo = '
                'resilient_nav_health_assessment.camera_health_feature_demo:main'
            ),
            (
                'camera_health_calibrate = '
                'resilient_nav_health_assessment.camera_health_calibrate:main'
            ),
            (
                'camera_health_baseline_report = '
                'resilient_nav_health_assessment.'
                'camera_health_baseline_report:main'
            ),
            (
                'camera_fault_feature_report = '
                'resilient_nav_health_assessment.'
                'camera_fault_feature_report:main'
            ),
            (
                'camera_health_monitor = '
                'resilient_nav_health_assessment.camera_health_monitor:main'
            ),
            (
                'camera_freeze_source = '
                'resilient_nav_health_assessment.camera_freeze_source:main'
            ),
            (
                'camera_health_watch = '
                'resilient_nav_health_assessment.camera_health_watch:main'
            ),
        ],
    },
)
