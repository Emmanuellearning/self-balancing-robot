import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'self_balancing_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='emmanuel',
    maintainer_email='emmanuel@todo.todo',
    description='Simple Self-Balancing Robot simulation in ROS 2 Jazzy and Gazebo',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pid_controller_node = self_balancing_sim.pid_controller_node:main',
            'pid_gain_solver = self_balancing_sim.pid_gain_solver:main',
        ],
    },
)
