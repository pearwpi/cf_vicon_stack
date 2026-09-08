import os
from glob import glob

from setuptools import setup

package_name = 'crazyflie_ros'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cs',
    maintainer_email='sriram.gaddipati@gmail.com',
    description='ROS 2 driver and teleop for a Vicon-fed Crazyflie 2.1+.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'crazyflie_server = crazyflie_ros.crazyflie_server:main',
            'teleop = crazyflie_ros.teleop_node:main',
        ],
    },
)
