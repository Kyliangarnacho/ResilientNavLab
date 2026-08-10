from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'resilient_nav_fault_injection'

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
        (os.path.join('share', package_name, 'config', 'scenarios'),
            glob(os.path.join('config', 'scenarios', '*.yaml'))),
        (os.path.join('share', package_name, 'rviz'),
            glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kylian',
    maintainer_email='296032282@qq.com',
    description='Minimal sensor fault injection nodes for resilient navigation.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            (
                'imu_fault_injector = '
                'resilient_nav_fault_injection.imu_fault_injector:main'
            ),
            (
                'imu_bias_injector = '
                'resilient_nav_fault_injection.imu_bias_injector:main'
            ),
            (
                'wheel_fault_injector = '
                'resilient_nav_fault_injection.wheel_fault_injector:main'
            ),
            (
                'scan_fault_injector = '
                'resilient_nav_fault_injection.scan_fault_injector:main'
            ),
            (
                'fault_probe = '
                'resilient_nav_fault_injection.fault_probe:main'
            ),
            (
                'phase5_record_bag = '
                'resilient_nav_fault_injection.phase5_record_bag:main'
            ),
            (
                'phase5_replay_bag = '
                'resilient_nav_fault_injection.phase5_replay_bag:main'
            ),
            (
                'manual_fault_event = '
                'resilient_nav_fault_injection.manual_fault_event:main'
            ),
        ],
    },
)
