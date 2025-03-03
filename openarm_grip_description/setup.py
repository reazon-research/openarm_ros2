from setuptools import setup
import os
from glob import glob

package_name = "openarm_grip_description"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*")),
        (os.path.join("share", package_name, "meshes"), glob("meshes/*")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Thomason",
    maintainer_email="t95zhou@uwaterloo.ca",
    description="URDF models for OpenArm",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
