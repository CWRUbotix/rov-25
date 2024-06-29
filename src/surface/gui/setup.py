"""setup.py for the gui module."""
import os
from glob import glob

from setuptools import setup

PACKAGE_NAME = 'gui'

setup(
    name=PACKAGE_NAME,
    version='1.2.0',
    packages=[PACKAGE_NAME, os.path.join(PACKAGE_NAME, 'widgets'),
              os.path.join(PACKAGE_NAME, 'styles'),
              os.path.join(PACKAGE_NAME, 'gui_nodes', 'auxiliary_nodes'),
              os.path.join(PACKAGE_NAME, 'gui_nodes', 'event_nodes')],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', PACKAGE_NAME, 'launch'),
         glob('launch/*launch.[pxy][yma]*')),
        # Include all style files.
        (os.path.join('share', PACKAGE_NAME, 'styles'),
         glob('gui/styles/*.qss')),
        # Include all images.
        (os.path.join('share', PACKAGE_NAME, 'images'),
         glob('gui/images/*')),
        # Include all sounds.
        (os.path.join('share', PACKAGE_NAME, 'sounds'),
         glob('gui/sounds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Benjamin Poulin',
    maintainer_email='bwp18@case.edu',
    description='MATE ROV GUI and related ROS nodes',
    license='Apache License 2.0',
    tests_require=['pytest', 'pytest-qt', 'pytest-xvfb'],
    entry_points={
        'console_scripts': ['run_pilot = gui.pilot_app:run_gui_pilot',
                            'run_operator = gui.operator_app:run_gui_operator',
                            'run_timer = gui.gui_nodes.auxiliary_nodes.timer:run_timer'],
    },
)
