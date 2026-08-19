"""Startet den LDS-006-Knoten.

    ros2 launch ant_lidar lds006.launch.py
    ros2 launch ant_lidar lds006.launch.py port:=/dev/ttyUSB1 offset_deg:=90
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        # /dev/ttyAMA0 ist auf ant000test vom DDSM-Antrieb belegt -> USB
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('frame_id', default_value='laser'),
        DeclareLaunchArgument('mirror', default_value='true'),
        DeclareLaunchArgument('offset_deg', default_value='0'),
        DeclareLaunchArgument('only_complete', default_value='true'),
    ]
    return LaunchDescription(args + [
        Node(
            package='ant_lidar',
            executable='lidar_node',
            name='lds006',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'frame_id': LaunchConfiguration('frame_id'),
                'mirror': LaunchConfiguration('mirror'),
                'offset_deg': LaunchConfiguration('offset_deg'),
                'only_complete': LaunchConfiguration('only_complete'),
            }],
        ),
    ])
