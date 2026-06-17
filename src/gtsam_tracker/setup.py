import os
from glob import glob
from setuptools import setup

package_name = "gtsam_tracker"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    py_modules=[],
    scripts=["scripts/gtsam_tracker_node"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy", "gtsam", "scipy"],
    zip_safe=True,
    maintainer="multiview_prosthesis",
    maintainer_email="dev@multiview-prosthesis.org",
    description="GTSAM factor graph trajectory binder ROS 2 node with SE(3) helpers",
    license="MIT",
)
