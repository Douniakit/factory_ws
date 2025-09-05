
# Factory Robot Workspace

A ROS 2 workspace for forward and inverse kinematics calculations of a 7-DOF robotic arm system.

# Overview

This workspace provides a complete kinematics solution for the ABB robot mounted on a flange, featuring:

Forward Kinematics: Compute end-effector pose from joint angles

Inverse Kinematics: Calculate joint configurations for desired end-effector poses

Real-time Publishing: Live pose and joint state broadcasting via ROS 2 topics

# System Architecture

- 7-DOF Configuration: 1 linear axis + 6 revolute joints
- Transformation Chain: URDF-based kinematic modeling with mounting offsets
- Multiple IK Solvers: Damped Least Squares and Levenberg-Marquardt methods

# Workspace Architecture

factory_ws/

├── README.md

└── src/

    ├── factory_launch/
    
    ├── factory_robot_description/
    
    ├── linear_axis_description/
    
    └── jacobi_kinematics/
    
    
# Package Descriptions

- factory_launch: Package containing system launch files for coordinated startup of all components

- factory_robot_description: Package containing ABB IRB6700-150/320 robot description (URDF, meshes, parameters)

- linear_axis_description: Package containing the linear axis/flange description on which the ABB robot is mounted

- jacobi_kinematics: Package containing the integrated system kinematics (robot + linear axis), including: System integration (combining robot and linear axis) ,
Joint naming and parameter management, Forward and inverse kinematics calculations and Real-time pose publishing and IK solving



# ROS 2 Topics

/joint_states - Input joint positions

/jacobi_kinematics/current_pose - Current end-effector pose

/jacobi_kinematics/target_pose - Desired target pose

/jacobi_kinematics/ik_joint_states - Computed IK solutions


# Build the workspace

cd factory_ws
colcon build
source install/setup.bash

# Source and run

Terminal 1, launch system visualisation:

cd factory_ws
source install/setup.bash
ros2 launch robot_launch display_system.launch.py

Terminal 2, Run kinematics service:

cd factory_ws
source install/setup.bash
ros2 run jacobi_kinematics kinematics_service

the inverse kinematic solution can be view in the Terminal 2 
alternatively you can listen to the topic /jacobi_kinematics/ik_joint_states in a third terminal using

source install/setup.bash
ros2 topic echo /jacobi_kinematics/ik_joint_states 

# Dependencies

ROS 2 (Jazzy/Humble)
NumPy
SciPy
Python 3.8+
