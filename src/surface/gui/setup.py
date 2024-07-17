"""setup.py for the gui module."""

from pathlib import Path

from setuptools import setup

PACKAGE_NAME = 'gui'

setup(
    name=PACKAGE_NAME,
    version='1.2.0',
    packages=[
        PACKAGE_NAME,
        str(Path(PACKAGE_NAME) / 'widgets'),
        str(Path(PACKAGE_NAME) / 'widgets' / 'tabs'),
        str(Path(PACKAGE_NAME) / 'styles'),
        str(Path(PACKAGE_NAME) / 'auxiliary_nodes'),
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        # Include all launch files.
        (
            str(Path('share') / PACKAGE_NAME / 'launch'),
            [str(path) for path in Path('launch').glob('*launch.[pxy][yma]*')],
        ),
        # Include all style files.
        (
            str(Path('share') / PACKAGE_NAME / 'styles'),
            [str(path) for path in (Path('gui') / 'styles').glob('*.qss')],
        ),
        # Include all images.
        (
            str(Path('share') / PACKAGE_NAME / 'images'),
            [str(path) for path in (Path('gui') / 'images').glob('*')],
        ),
        # Include all sounds.
        (
            str(Path('share') / PACKAGE_NAME / 'sounds'),
            [str(path) for path in (Path('gui') / 'sounds').glob('*')],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Benjamin Poulin',
    maintainer_email='bwp18@case.edu',
    description='MATE ROV GUI and related ROS nodes',
    license='Apache License 2.0',
    tests_require=['pytest', 'pytest-qt', 'pytest-xvfb'],
    entry_points={
        'console_scripts': [
            'run_pilot = gui.pilot_app:run_gui_pilot',
            'run_operator = gui.operator_app:run_gui_operator',
            'run_timer = gui.auxiliary_nodes.timer:run_timer',
        ],
    },
)
