import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'xresto_drone'
    pkg_share = get_package_share_directory(pkg_name)

    # Path to your drone URDF
    drone_urdf_path = os.path.join(pkg_share, 'urdf', 'Final_Drone.urdf')
    with open(drone_urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # Path to the new red gate URDF
    gate_urdf_path = os.path.join(pkg_share, 'urdf', 'red_gate.urdf')

    # Robot State Publisher for the drone
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )

    # Launch Gazebo Server (Omit 'world' argument to default to empty world)
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'extra_gazebo_args': '--verbose'}.items()
    )

    # Launch Gazebo Client
    delayed_gzclient = TimerAction(
        period=2.5,
        actions=[
            ExecuteProcess(cmd=['gzclient', '--verbose'], output='screen')
        ]
    )

    # Spawn the drone at the origin
    spawn_drone = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', drone_urdf_path,
            '-entity', 'my_custom_drone',
            '-x', '0.0', '-y', '0.0', '-z', '6'
        ],
        output='screen'
    )

    # Spawn the red gate 3 meters in front of the drone
    spawn_gate = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', gate_urdf_path,
            '-entity', 'red_gate',
            '-x', '3.0', '-y', '0.0', '-z', '0'
        ],
        output='screen'
    )

    # Delay the spawning slightly to ensure Gazebo is fully loaded
    delayed_spawn_drone = TimerAction(period=8.0, actions=[spawn_drone])
    delayed_spawn_gate = TimerAction(period=10.0, actions=[spawn_gate])

    return LaunchDescription([
        rsp_node,
        gazebo_server,
        delayed_gzclient,
        delayed_spawn_drone,
        delayed_spawn_gate
    ])
