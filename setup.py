from setuptools import find_packages, setup

package_name = 'debug_tool'

setup(
    name=package_name,
    version='1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='stereolabs',
    maintainer_email='support@stereolabs.com',
    description='Package allowing to sync data between a ROS Wrapper and a ROS2 bag',
    license='Stereolabs',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'sync_node = debug_tool.sync_node:main',
        'control_svo_node = debug_tool.control_svo_node:main'
        ],
    },
)
