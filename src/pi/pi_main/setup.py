"""setup.py for pi_main module."""

from pathlib import Path

from setuptools import setup

PACKAGE_NAME = 'pi_main'


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
            str(Path('share') / PACKAGE_NAME / 'udev_rules'),
            [str(path) for path in Path('udev_rules').glob('*')],
        ),
        (
            str(Path('share') / PACKAGE_NAME / 'services'),
            [str(path) for path in Path('services').glob('*')],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Michael Carlstrom',
    maintainer_email='rmc170@case.edu',
    description='Mate ROV Main code launcher',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['install = pi_main.run_on_boot:main'],
    },
)
