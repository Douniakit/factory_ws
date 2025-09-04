import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    package_name = 'linear_axis_description'
    pkg_share = get_package_share_directory(package_name)
    
    # Update this with your actual URDF filename
    urdf_file = 'linear_axis.urdf'  # Change this to your URDF filename
    robot_description_path = os.path.join(pkg_share, 'urdf', urdf_file)
    
    # Read URDF file
    with open(robot_description_path, 'r') as infp:
        robot_description = infp.read()
    
    print(f"Loading Linear Axis URDF from: {robot_description_path}")
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )
    
    # Joint State Publisher GUI
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    # RViz
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'linear_axis_config.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else []
    )
    
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])