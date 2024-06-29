"""setup.py for pi_main module."""
import os
from glob import glob

from setuptools import setup

PACKAGE_NAME = 'pi_main'


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
         glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', PACKAGE_NAME, 'udev_rules'),
         glob('udev_rules/*')),
        (os.path.join('share', PACKAGE_NAME, 'services'),
         glob('services/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Michael Carlstrom',
    maintainer_email='rmc170@case.edu',
    description='Mate ROV Main code launcher',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'install = pi_main.run_on_boot:main'
        ],
    },
)
