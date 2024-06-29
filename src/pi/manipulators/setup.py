"""setup.py for manipulators module."""
import os
from glob import glob

from setuptools import setup

PACKAGE_NAME = 'manipulators'

setup(
    name=PACKAGE_NAME,
    version='1.2.0',
    packages=[PACKAGE_NAME],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', PACKAGE_NAME, 'launch'),
         glob('launch/*launch.[pxy][yma]*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Georgia Martinez, Michael Carlstrom',
    maintainer_email='gcm49@case.edu, rmc170@case.edu',
    description='Code for manipulators',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'manipulators = manipulators.manipulator_node:main',
            'dry_run_manipulators = manipulators.manipulator_dry_run:main',
            'valve_manipulator = manipulators.valve_manipulator_node:main',
            'dry_run_valve_manipulator = manipulators.valve_manipulator_dry_run:main',
            'lgpio_manipulator = manipulators.lgpio_manipulator:main',
            'dry_run_lgpio_manipulator = manipulators.lgpio_manipulator_dry_run:main'
        ],
    },
)
