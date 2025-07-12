#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Locate the directory where our RViz config is stored
    cddp_mpc_package_share = get_package_share_directory('cddp_mpc')
    rviz_config_file = os.path.join(cddp_mpc_package_share, 'rviz', 'simple_launch.rviz')

    # Node for publishing the grid map
    grid_map_node = Node(
        package='cddp_mpc',
        executable='hardcoded_map_node',
        name='hardcoded_map_node',
        output='screen'
    )

    # Node for RViz2 with our configuration file
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # Node for the unicycle robot simulation.
    # Parameters: robot_id, initial x, y, and yaw.
    unicycle_node = Node(
        package='cddp_mpc',
        executable='forklift_robot_node',
        name='forklift_robot_node',
        output='screen',
        parameters=[{
            'robot_id': 'robot_1',
            'init_x': 0.0,
            'init_y': 0.0,
            'init_yaw': 0.0
        }]
    )

    return LaunchDescription([
        grid_map_node,
        rviz_node,
        unicycle_node,
    ])
