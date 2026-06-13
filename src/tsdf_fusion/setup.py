import os
from glob import glob
from setuptools import setup

package_name = "tsdf_fusion"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    py_modules=[],
    entry_points={
        "console_scripts": [
            "tsdf_fusion_node = tsdf_fusion.tsdf_fusion_node:main",
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
    description="On-demand TSDF fusion triggered at grasp time (MobileSAM + keyframes)",
    license="MIT",
)
