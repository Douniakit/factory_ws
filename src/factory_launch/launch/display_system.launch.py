import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xml.etree.ElementTree as ET

def generate_launch_description():
    # Get package directories
    robot_description_dir = get_package_share_directory('factory_robot_description')
    linear_axis_description_dir = get_package_share_directory('linear_axis_description')
    
    # Paths to URDF files
    robot_urdf = os.path.join(robot_description_dir, 'urdf', 'abb_irb6700_150_320.urdf')
    linear_axis_urdf = os.path.join(linear_axis_description_dir, 'urdf', 'linear_axis.urdf')
    
    # Read and parse URDF files
    with open(robot_urdf, 'r') as f:
        robot_xml = f.read()
    
    with open(linear_axis_urdf, 'r') as f:
        linear_axis_xml = f.read()
    
    # Parse XML
    robot_tree = ET.fromstring(robot_xml)
    linear_axis_tree = ET.fromstring(linear_axis_xml)
    
    # Create new root robot element
    combined_robot = ET.Element('robot')
    combined_robot.set('name', 'robot_with_linear_axis')
    
    # Copy all elements from linear axis (keep original names)
    for child in linear_axis_tree:
        if child.tag in ['link', 'joint', 'material', 'gazebo']:
            combined_robot.append(child)
    
    # Process robot elements with renamed links
    for child in robot_tree:
        if child.tag == 'link':
            # Clone the element
            new_link = ET.fromstring(ET.tostring(child))
            # Prefix all robot links with 'robot_'
            old_name = new_link.get('name')
            new_name = 'robot_' + old_name
            new_link.set('name', new_name)
            combined_robot.append(new_link)
            
        elif child.tag == 'joint':
            # Clone the element
            new_joint = ET.fromstring(ET.tostring(child))
            
            # Update parent and child link references
            parent = new_joint.find('parent')
            child_elem = new_joint.find('child')
            
            if parent is not None:
                old_parent = parent.get('link')
                parent.set('link', 'robot_' + old_parent)
                    
            if child_elem is not None:
                old_child = child_elem.get('link')
                child_elem.set('link', 'robot_' + old_child)
            
            combined_robot.append(new_joint)
            
        elif child.tag == 'material':
            combined_robot.append(child)
            
        elif child.tag == 'gazebo':
            # Clone and update gazebo references
            new_gazebo = ET.fromstring(ET.tostring(child))
            if 'reference' in new_gazebo.attrib:
                old_ref = new_gazebo.get('reference')
                new_gazebo.set('reference', 'robot_' + old_ref)
            combined_robot.append(new_gazebo)
    
    # Add connecting joint between linear axis flange and robot base
    connecting_joint = ET.SubElement(combined_robot, 'joint')
    connecting_joint.set('name', 'linear_axis_to_robot')
    connecting_joint.set('type', 'fixed')
    
    parent = ET.SubElement(connecting_joint, 'parent')
    parent.set('link', 'flange')  # Linear axis end link
    
    child = ET.SubElement(connecting_joint, 'child')
    child.set('link', 'robot_base_link')  # Robot base link (with prefix)
    
    origin = ET.SubElement(connecting_joint, 'origin')
    origin.set('xyz', '0 0 0.1')  # Raise robot 10cm above linear axis flange
    origin.set('rpy', '0 0 0')
    
    # Convert to string
    robot_description = ET.tostring(combined_robot, encoding='unicode')
    
    # Save for debugging
    debug_file = '/tmp/combined_robot.urdf'
    with open(debug_file, 'w') as f:
        f.write(robot_description)
    print(f"Debug: Combined URDF saved to {debug_file}")
    
    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )
    
    # Joint state publisher GUI for testing
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    # RViz node
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )
    
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        rviz
    ])