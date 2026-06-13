import os
from glob import glob
from setuptools import setup

package_name = "keyframe_buffer"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    py_modules=[],
    entry_points={
        "console_scripts": [
            "keyframe_buffer_node = keyframe_buffer.keyframe_buffer_node:main",
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
    description="Spatial-gated keyframe storage with unorganized-safe cloud handling",
    license="MIT",
)
