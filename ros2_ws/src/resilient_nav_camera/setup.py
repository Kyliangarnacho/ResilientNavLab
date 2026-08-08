from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'resilient_nav_camera'


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
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kylian',
    maintainer_email='296032282@qq.com',
    description='C920 USB camera baseline and metadata probe for ResilientNavLab.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'c920_probe = resilient_nav_camera.c920_probe:main',
            'calibration_reuse_validator = '
            'resilient_nav_camera.calibration_reuse_validator:main',
            'rectification_probe = '
            'resilient_nav_camera.rectification_probe:main',
        ],
    },
)
