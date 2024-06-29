import os
from glob import glob

from setuptools import setup

PACKAGE_NAME = 'flight_control'

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
    maintainer='benjamin',
    maintainer_email='bwp18@case.edu',
    description='Mate ROV sub movement controllers',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'manual_control_node = flight_control.manual_control_node:main',
            'keyboard_control_node = flight_control.keyboard_control_node:main',
            'auto_docking_node = flight_control.auto_docking_node:main',
            'control_inverter_node = flight_control.control_inverter_node:main',
            'multiplexer_node = flight_control.multiplexer:main'
        ],
    },
)
