from pathlib import Path

from setuptools import setup

PACKAGE_NAME = 'photosphere'

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
    maintainer='Shannon Griswold',
    maintainer_email='svg33@case.edu',
    description='Creates a photosphere and streams fisheye cameras',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run_photosphere = photosphere.photosphere:main',
            'photosphere_driver = photosphere.photosphere_driver_node:main',
        ],
    },
)
