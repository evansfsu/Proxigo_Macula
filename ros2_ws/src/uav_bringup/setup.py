from setuptools import setup
import os
from glob import glob

package_name = "uav_bringup"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("uav_bringup/launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
)
