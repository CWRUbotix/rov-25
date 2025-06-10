from pathlib import Path

from setuptools import find_packages, setup

PACKAGE_NAME = 'luxonis_cam'

setup(
    name=PACKAGE_NAME,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        (
            str(Path('share') / PACKAGE_NAME / 'launch'),
            [str(path) for path in Path('launch').glob('*launch.[pxy][yma]*')],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='noah',
    maintainer_email='noah@mollerstuen.com',
    description='Driver for Luxonis stereo camera',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['luxonis_cam_driver = luxonis_cam.cam_driver:main','record_cam = luxonis_cam.record_cam:main']
    },
)
