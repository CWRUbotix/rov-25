"""setup.py for camera_streamer module."""
import os
from glob import glob

from setuptools import setup

PACKAGE_NAME = 'camera_streamer'

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
    maintainer='Noah Mollerstuen',
    maintainer_email='noah@mollerstuen.com',
    description='contains camera information as well as camera launch files',
    license='Apaches License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
