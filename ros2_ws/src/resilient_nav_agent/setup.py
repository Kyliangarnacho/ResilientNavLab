from setuptools import find_packages, setup


package_name = 'resilient_nav_agent'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'agent-core>=0.1.0',
        'pydantic>=2.8',
        'setuptools',
    ],
    zip_safe=False,
    maintainer='Kylian',
    maintainer_email='296032282@qq.com',
    description='Offline Robot Diagnostic Agent domain for ResilientNavLab.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'ra1a_fake_benchmark = '
            'resilient_nav_agent.benchmark.runner:main',
        ],
    },
)
