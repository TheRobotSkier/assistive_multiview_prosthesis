from setuptools import find_packages, setup

package_name = 'pipeline_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel',
    maintainer_email='daniel@example.com',
    description='Pipeline state machine orchestrating the grasp workflow',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pipeline_manager_node = pipeline_manager.pipeline_manager_node:main',
            'mock_cloud_publisher = pipeline_manager.mock_cloud_publisher:main',
        ],
    },
)
