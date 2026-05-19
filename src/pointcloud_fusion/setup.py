import os
from glob import glob
from setuptools import setup

package_name = "pointcloud_fusion"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    py_modules=[],
    entry_points={
        "console_scripts": [
            "pointcloud_fusion_node = pointcloud_fusion.pointcloud_fusion_node:main",
        ],
    },
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="multiview_prosthesis",
    maintainer_email="dev@multiview-prosthesis.org",
    description="Fuses dual-camera point clouds into a unified, filtered cloud in world frame",
    license="MIT",
)
