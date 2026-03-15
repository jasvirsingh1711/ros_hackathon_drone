#!/usr/bin/env python3
"""
Main launch file for autonomous drone gate navigation mission
Launches Gazebo with gate course, drone, and all autonomous control nodes
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'xresto_drone'
    pkg_share = get_package_share_directory(pkg_name)

    # Paths
    drone_urdf_path = os.path.join(pkg_share, 'urdf', 'Final_Drone.urdf')
    aerial_nav_world = os.path.join(pkg_share, 'worlds', 'aerial_nav.world')
    
    # Read drone URDF
    with open(drone_urdf_path, 'r') as f:
        robot_desc = f.read()

    # ===== GAZEBO SIMULATION =====
    # Start Gazebo with aerial navigation world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world': aerial_nav_world,
            'verbose': 'true',
            'pause': 'false'
        }.items()
    )

    # ===== ROBOT STATE PUBLISHER =====
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )

    # ===== SPAWN DRONE =====
    spawn_drone = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', drone_urdf_path,
            '-entity', 'autonomous_drone',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.8'
        ],
        output='screen'
    )

    # Delay spawning to ensure Gazebo is loaded (gate models removed for faster startup)
    delayed_spawn_drone = TimerAction(period=15.0, actions=[spawn_drone])

    # ===== AUTONOMOUS CONTROL NODES =====
    
    # 1. Hover Controller - Maintains altitude and attitude stability
    hover_controller = Node(
        package='xresto_drone',
        executable='hover_node.py',
        name='hover_controller',
        output='screen'
    )

    # 2. Gate Detector - Detects gates from camera and depth images
    gate_detector = Node(
        package='xresto_drone',
        executable='gate_detector',
        name='gate_detector',
        output='screen'
    )

    # 2B. Landing Detector - Detects red landing pad for final descent
    landing_detector = Node(
        package='xresto_drone',
        executable='landing_detector.py',
        name='landing_detector',
        output='screen'
    )

    # 3. Gate Navigator - Manages gate sequencing and transitions
    gate_navigator = Node(
        package='xresto_drone',
        executable='gate_navigator.py',
        name='gate_navigator',
        output='screen'
    )

    # 4. Trajectory Planner - Plans collision-free paths
    trajectory_planner = Node(
        package='xresto_drone',
        executable='trajectory_planner.py',
        name='trajectory_planner',
        output='screen'
    )

    # ===== RViz VISUALIZATION =====
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'drone_viz.rviz')
    
    # RViz visualization node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}]
    ) if os.path.exists(rviz_config_path) else None

    # ===== JOINT STATE PUBLISHER (for visualization of propeller rotation) =====
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # Build launch description
    launch_desc = [
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        delayed_spawn_drone,
        # Delay autonomous nodes to ensure drone is spawned
        TimerAction(period=16.0, actions=[hover_controller]),
        TimerAction(period=16.5, actions=[gate_detector]),
        TimerAction(period=16.7, actions=[landing_detector]),
        TimerAction(period=17.0, actions=[gate_navigator]),
        TimerAction(period=17.5, actions=[trajectory_planner]),
    ]

    if rviz_node:
        launch_desc.append(TimerAction(period=18.5, actions=[rviz_node]))

    return LaunchDescription(launch_desc)
