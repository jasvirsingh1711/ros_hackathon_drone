import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Setup paths
    pkg_name = 'xresto_drone' # Make sure this matches your package name
    pkg_share = get_package_share_directory(pkg_name)
    urdf_path = os.path.join(pkg_share, 'urdf', 'Final_Drone.urdf')

    # 2. Read the URDF file
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # 3. Robot State Publisher (RSP)
    # This takes the joint positions and URDF to create the 3D transforms (TF)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # 4. Joint State Publisher GUI (JSP GUI)
    # This opens the window with sliders so you can move the propellers
    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # 5. RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        rsp_node,
        jsp_gui_node,
        rviz_node
    ])