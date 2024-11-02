from pathlib import Path

from setuptools import setup

PACKAGE_NAME = 'transceiver'

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
    maintainer='ericy',
    maintainer_email='ery12@case.edu',
    description='Mate ROV transceiver communication code',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial = transceiver.serial_reader:main',
        ],
    },
)
