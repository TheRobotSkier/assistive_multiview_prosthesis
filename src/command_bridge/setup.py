import os
from glob import glob
from setuptools import setup, find_packages

package_name = "command_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Daniel",
    maintainer_email="daniel@example.com",
    description="Bridge between ros2_control-style command topics and the raw Mia Hand driver services",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "command_bridge_node = command_bridge.command_bridge_node:main",
        ],
    },
)
