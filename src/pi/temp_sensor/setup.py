from pathlib import Path

from setuptools import setup

PACKAGE_NAME = 'temp_sensor'

setup(
    name=PACKAGE_NAME,
    version='1.2.0',
    packages=[PACKAGE_NAME],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        # Include all launch files.
        (
            str(Path('share') / PACKAGE_NAME / 'launch'),
            [str(path) for path in Path('launch').glob('*launch.[pxy][yma]*')],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Benjamin Poulin',
    maintainer_email='bwp18@case.edu',
    description='Temperature sensor',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'temp_sensor = temp_sensor.temp_sensor:main',
            'dry_run = temp_sensor.temp_sensor_dry_run:main',
        ],
    },
)
