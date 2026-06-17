import os
from glob import glob
from setuptools import setup

package_name = "cross_camera_features"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    py_modules=[],
    scripts=["scripts/sift_feature_node"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="multiview_prosthesis",
    maintainer_email="dev@multiview-prosthesis.org",
    description="SIFT-based cross-camera 3D-3D alignment (head/arm) for the GTSAM tracker",
    license="MIT",
)
