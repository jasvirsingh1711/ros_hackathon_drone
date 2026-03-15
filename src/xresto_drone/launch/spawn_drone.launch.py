import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Specify the name of your package
    pkg_name = 'xresto_drone'

    # Find the paths to the package and the URDF file
    pkg_share = get_package_share_directory(pkg_name)
    urdf_path = os.path.join(pkg_share, 'urdf', 'Final_Drone.urdf')

    # Read the URDF file
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # 1. Start Robot State Publisher (Broadcasts your drone's joints and links to ROS)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # 2. Include the standard Gazebo Classic launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    # 3. Spawn the drone into Gazebo using the robot_description topic
    # We set '-z 0.5' to drop the drone from half a meter in the air
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', 
                   '-entity', 'my_custom_drone', 
                   '-z', '0.5'],
        output='screen'
    )

    # Add this node to your launch description
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    return LaunchDescription([
        rsp_node,
        gazebo,
        spawn_entity,
        joint_state_publisher,
    ])