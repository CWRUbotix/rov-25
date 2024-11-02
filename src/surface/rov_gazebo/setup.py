"""setup.py for the rov_gazebo module."""

from pathlib import Path

from setuptools import setup

PACKAGE_NAME = 'rov_gazebo'

setup(
    name=PACKAGE_NAME,
    version='1.2.0',
    packages=[PACKAGE_NAME],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        (
            str(Path('share') / PACKAGE_NAME / 'launch'),
            [str(path) for path in Path('launch').glob('*launch.[pxy][yma]*')],
        ),
        (
            str(Path('share') / PACKAGE_NAME / 'description'),
            [str(path) for path in Path('description').glob('*')],
        ),
        (
            str(Path('share') / PACKAGE_NAME / 'config'),
            [str(path) for path in Path('config').glob('*')],
        ),
        (
            str(Path('share') / PACKAGE_NAME / 'worlds'),
            [str(path) for path in Path('worlds').glob('*')],
        ),
        (
            str(Path('share') / PACKAGE_NAME / 'meshes'),
            [str(path) for path in Path('meshes').glob('*')],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Noah Mollerstuen',
    maintainer_email='nrm98@case.edu',
    description='MATE ROV simulation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
