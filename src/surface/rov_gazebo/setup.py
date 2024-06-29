"""setup.py for the rov_gazebo module."""
import os
from glob import glob

from setuptools import setup

PACKAGE_NAME = "rov_gazebo"

setup(
    name=PACKAGE_NAME,
    version="1.2.0",
    packages=[PACKAGE_NAME],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
        (os.path.join("share", PACKAGE_NAME, "launch"), glob("launch/*.py")),
        (os.path.join("share", PACKAGE_NAME, "description"), glob("description/*")),
        (os.path.join("share", PACKAGE_NAME, "config"), glob("config/*")),
        (os.path.join("share", PACKAGE_NAME, "worlds"), glob("worlds/*")),
        (os.path.join("share", PACKAGE_NAME, "meshes"), glob("meshes/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Seongmin Jung",
    maintainer_email="sxj754@case.edu",
    description="MATE ROV simulation",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
