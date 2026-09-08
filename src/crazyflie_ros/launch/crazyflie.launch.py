"""Bring up the Vicon bridge, the Crazyflie driver, and optionally teleop.

    ros2 launch crazyflie_ros crazyflie.launch.py
    ros2 launch crazyflie_ros crazyflie.launch.py teleop:=false
    ros2 launch crazyflie_ros crazyflie.launch.py no_fly:=true
    ros2 launch crazyflie_ros crazyflie.launch.py vicon:=false   # bridge already running

Teleop needs a terminal it can put into raw mode, so it is launched with
output='screen' and emulate_tty. If keys do not register, run it in its own
terminal instead:  ros2 run crazyflie_ros teleop --ros-args --params-file <cfg>
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('crazyflie_ros'),
                       'config', 'crazyflie.yaml')

    args = [
        DeclareLaunchArgument('config', default_value=cfg),
        DeclareLaunchArgument('teleop', default_value='true'),
        DeclareLaunchArgument('vicon', default_value='true'),
        DeclareLaunchArgument('no_fly', default_value='false'),
        DeclareLaunchArgument('vicon_hostname', default_value='192.168.10.1'),
    ]
    config = LaunchConfiguration('config')

    vicon = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('vicon_receiver'),
            'launch', 'client.launch.py')),
        launch_arguments={'hostname': LaunchConfiguration('vicon_hostname')}.items(),
        condition=IfCondition(LaunchConfiguration('vicon')),
    )

    server = Node(
        package='crazyflie_ros', executable='crazyflie_server',
        name='crazyflie_server', output='screen',
        parameters=[config, {'no_fly': LaunchConfiguration('no_fly')}],
    )

    teleop = Node(
        package='crazyflie_ros', executable='teleop',
        name='crazyflie_teleop', output='screen', emulate_tty=True,
        parameters=[config],
        condition=IfCondition(LaunchConfiguration('teleop')),
    )

    return LaunchDescription(args + [vicon, server, teleop])
