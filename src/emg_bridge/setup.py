from setuptools import setup, find_packages

package_name = 'emg_bridge'

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
    description='MindRove EMG classifier with ROS 2 bridge for gesture detection',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run_classifier = emg_bridge.run_classifier_entry:main',
            'collect_data = emg_bridge.collect_data_entry:main',
            'train = emg_bridge.train_entry:main',
            'emg_grasp_controller = emg_bridge.emg_grasp_controller:main',
        ],
    },
)
