from setuptools import setup
import os

package_name = "uav_guidance"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    scripts=[
        "scripts/offboard_controller.py"
    ],
    entry_points={
        "console_scripts": [
            "offboard_hello = uav_guidance.offboard_hello:main",
        ],
    },
)

