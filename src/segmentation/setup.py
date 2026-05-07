from setuptools import setup, find_packages

package_name = 'segmentation_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel',
    maintainer_email='daniel@example.com',
    description='ROS 2 bridge node for InterObject3D point cloud segmentation',
    license='MIT',
    entry_points={
        'console_scripts': [
            'segmentation_ros2_node = segmentation_bridge.segmentation_ros2_node:main',
        ],
    },
)
