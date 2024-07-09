import os
from glob import glob

from setuptools import find_packages, setup

PACKAGE_NAME = 'vehicle_manager'

setup(
    name=PACKAGE_NAME,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        # Include all launch files.
        (
            os.path.join('share', PACKAGE_NAME, 'launch'),
            glob('launch/*launch.[pxy][yma]*'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='noah',
    maintainer_email='noah@mollerstuen.com',
    description='Surface packaage to manage vehicle state updates',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'connection_manager_node = vehicle_manager.connection_manager_node:main'
        ],
    },
)
