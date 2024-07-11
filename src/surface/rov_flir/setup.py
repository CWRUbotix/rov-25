from pathlib import Path

from setuptools import setup

PACKAGE_NAME = 'rov_flir'

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
        (
            str(Path('share') / PACKAGE_NAME / 'config'),
            [str(path) for path in Path('config').glob('config/*')],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Michael Carlstrom',
    maintainer_email='rmc170@case.edu',
    description='Boilerplate for calling standard flir launch file.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'flir_watchdog = rov_flir.flir_watchdog:main',
        ],
    },
)
