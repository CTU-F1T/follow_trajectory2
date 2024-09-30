#!/usr/bin/env python
# setup.py
"""Install script for ROS1 catkin / ROS2 ament_python."""

from setuptools import setup

package_name = 'follow_trajectory2'

setup(
    name = package_name,
    version = '0.0.0',
    # The second item is probably needed when not installing using simlink
    packages = [package_name, package_name+".controllers"],
    data_files = [
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires = ['setuptools'],
    zip_safe = True,
    author = 'Jaroslav Klapálek',
    author_email = 'klapajar@fel.cvut.cz',
    maintainer = 'F1tenth CTU Community',
    maintainer_email = 'f1tenth@rtime.felk.cvut.cz',
    description = 'ROS package for following a planned trajectory of the car.',
    license = 'GPLv3',
    tests_require = ['pytest'],
    entry_points = {
        'console_scripts': [
            'run = follow_trajectory2.run:main'
        ],
    },
)
