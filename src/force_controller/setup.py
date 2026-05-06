import os
from glob import glob
from setuptools import setup, find_packages

package_name = "force_controller"

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
    maintainer="multiview_prosthesis",
    maintainer_email="dev@multiview-prosthesis.org",
    description="Force controller for Mia Hand grasp regulation",
    license="MIT",
    entry_points={
        "console_scripts": [
            "force_controller_node = force_controller.force_controller_node:main",
        ],
    },
)
