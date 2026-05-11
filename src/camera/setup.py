import os
from glob import glob
from setuptools import setup

package_name = "camera"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    py_modules=[],
    entry_points={
        "console_scripts": [
            "charuco_tf_node = camera.charuco_tf_node:main",
            "hand_pose_publisher = camera.hand_pose_publisher:main",
            "pointcloud_relay_node = camera.pointcloud_relay_node:main",
            "cloud_snapshot_node = camera.cloud_snapshot_node:main",
            "pointcloud_fuser_node = camera.pointcloud_fuser_node:main",
            "pointcloud_merger_node = camera.pointcloud_merger_node:main",
            "cam2_hand_tracker_node = camera.cam2_hand_tracker_node:main",
        ],
    },
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "opencv-python", "numpy", "cv-bridge"],
    zip_safe=True,
    maintainer="multiview_prosthesis",
    maintainer_email="dev@multiview-prosthesis.org",
    description="RealSense D435 camera launch configuration and charuco calibration",
    license="MIT",
)
