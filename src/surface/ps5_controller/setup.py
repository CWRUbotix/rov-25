"""setup.py for the ps5_controller module."""
import os
from glob import glob

from setuptools import setup

PACKAGE_NAME = 'ps5_controller'

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
    maintainer='Michael Carlstrom',
    maintainer_email='rmc170@case.edu',
    description='Boilerplate for calling standard joystick launch file.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={},
)
