from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/robot_control.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xotn',
    maintainer_email='james190414@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_control_node = robot_control.robot_control_node:main'
        ],
    },
)
