import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'carla_adas_stack'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maxim Jakinin',
    maintainer_email='mjakinin@users.noreply.github.com',
    description='Modular Autonomous Driving and ADAS Stack in ROS 2 for CARLA Simulator',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'carla_interface_node = carla_adas_stack.carla_interface_node:main',
            'stanley_controller_node = carla_adas_stack.stanley_controller_node:main',
            'aeb_node = carla_adas_stack.aeb_node:main',
            'frenet_planner_node = carla_adas_stack.frenet_planner_node:main',
            'state_estimator_node = carla_adas_stack.state_estimator_node:main',
        ],
    },
)
