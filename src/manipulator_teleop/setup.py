import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'manipulator_teleop'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CVSP Engineer',
    maintainer_email='engineer@robotics.dev',
    description='Real-Time Cartesian Teleoperation and Autonomous Sequencer',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'teleop_twist_keyboard = manipulator_teleop.teleop_twist_keyboard:main',
            'pick_and_place_node = manipulator_teleop.pick_and_place_node:main',
            'benchmark_planners = manipulator_teleop.benchmark_planners:main',
        ],
    },
)
