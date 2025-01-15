#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('visioscan'),
        'config',
        'params.yaml'
    )

    rviz_config = os.path.join(
        get_package_share_directory('visioscan'),
        'rviz',
        'visioscan.rviz'
    )

    visioscan_node = Node(
        package = 'visioscan',
        namespace = 'bea_power',
        executable = 'visioscan_node',
        parameters = [config]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    ld.add_action(visioscan_node)
    ld.add_action(rviz_node)

    return ld
