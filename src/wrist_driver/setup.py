import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'wrist_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel',
    maintainer_email='daniel@example.com',
    description='Dynamixel wrist motor driver for the prosthesis',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wrist_driver_node = wrist_driver.wrist_driver_node:main',
            'wrist_driver_sim_node = wrist_driver.wrist_driver_sim_node:main',
            'emg_wrist_controller = wrist_driver.emg_wrist_controller:main',
        ],
    },
)
