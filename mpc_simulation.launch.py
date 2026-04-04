#!/usr/bin/env python3
"""
Unified launch file for MPC simulation with PX4 SITL
This launches all components needed for the MPC simulation in one command
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_dir = get_package_share_directory('quadrotor_mpc')
    
    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
    )
    
    auto_takeoff_arg = DeclareLaunchArgument(
        'auto_takeoff',
        default_value='true',
        description='Automatically takeoff after launch'
    )
    
    vehicle_id_arg = DeclareLaunchArgument(
        'vehicle_id',
        default_value='px4_robot',
        description='Vehicle ID for namespacing'
    )
    
    return LaunchDescription([
        # Launch arguments
        use_rviz_arg,
        auto_takeoff_arg,
        vehicle_id_arg,
        # Process manager (PX4 SITL + micro-XRCE-DDS)
        Node(
            package='quadrotor_mpc',
            executable='processes.py',
            name='processes',
            prefix='gnome-terminal --',
            output='screen'
        ),
        
        # Note: MPC control is handled by the C++ mpc_node below
        # Removed Python mpc_control.py to avoid conflicts
        
        # PX4 visualizer node (with delay to ensure PX4 is ready)
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'quadrotor_mpc', 'px4_visualizer'],
                    shell=False,
                    output='screen'
                )
            ]
        ),
        
        # RViz2 for visualization (with 3 second delay to ensure nodes are ready)
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', os.path.join(package_dir, 'config', 'mpc_visualization.rviz')],
                    output='screen',
                    condition=IfCondition(LaunchConfiguration('use_rviz'))
                )
            ]
        ),
        
        # MPC node
        Node(
            package='quadrotor_mpc',
            executable='mpc_node',
            name='mpc_node',
            output='screen',
            parameters=[{
                'vehicle_id': LaunchConfiguration('vehicle_id'),
                # 'mass_': 1.5,
                # 'arm_length_': 0.25,
                # 'Ixx_': 0.029,
                # 'Iyy_': 0.029,
                # 'Izz_': 0.055,
                # 'f_min_': 0.0,
                # 'f_max_': 8.0,
                # 'processing_frequency_': 50.0,
                # 'horizon_': 20,
                # 'timestep_': 0.02,
            }],
            # Note: Remappings would need to be dynamic based on vehicle_id
            # For now, we'll use the default px4_robot namespace
        ),
        
        # Takeoff script (delayed start)
        TimerAction(
            period=10.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'quadrotor_mpc', 'takeoff.py'],
                    shell=False,
                    output='screen',
                    condition=IfCondition(LaunchConfiguration('auto_takeoff'))
                )
            ]
        ),
    ])