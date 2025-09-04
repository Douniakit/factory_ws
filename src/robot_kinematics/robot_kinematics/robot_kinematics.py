"""Main robot forward kinematics class for the 7-DOF system"""

import numpy as np
from typing import Optional, Tuple

class RobotKinematics:
    """Main robot kinematics class for the 7-DOF system"""
    
    def __init__(self, logger=None):
        from .robot_parameters import RobotParameters
        self.params = RobotParameters()
        self.logger = logger
        
        # Current robot state
        self.current_joint_positions = np.zeros(7)
        self.state_valid = False
        
        if logger:
            logger.info("Robot Kinematics initialized")
    
    def update_joint_state(self, joint_positions: np.ndarray, 
                          joint_velocities: Optional[np.ndarray] = None,
                          joint_efforts: Optional[np.ndarray] = None) -> bool:
        """Update the robot's current joint state"""
        
        if len(joint_positions) != 7:
            return False
        
        self.current_joint_positions = joint_positions.copy()
        self.state_valid = True
        return True
        
    def forward_kinematics(self, joint_positions: Optional[np.ndarray] = None) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Compute forward kinematics using transformation matrices"""
        
        if joint_positions is not None:
            if len(joint_positions) != 7:
                return None, None
            temp_positions = self.current_joint_positions.copy()
            self.current_joint_positions = joint_positions
        else:
            if not self.state_valid:
                return None, None
            temp_positions = None

        try:
            # Build transformation matrices
            T = np.eye(4)
            
            # 1. Linear axis transformation
            T[0, 3] = self.current_joint_positions[0]
            
            # 2. Robot mounting offset
            mounting_offset = self.params._get_connection_transform()['total_offset']
            T[0:3, 3] += mounting_offset
            
            # 3. Robot joint transformations
            robot_joints = self.current_joint_positions[1:]
            
            # Joint transformations with hardcoded parameters
            transforms = [
                ([0, 0, 0.780], [0, 0, 1], robot_joints[0]),  # Base rotation
                ([0.320, 0, 0], [0, 1, 0], robot_joints[1]),  # Shoulder
                ([0, 0, 1.280], [0, 1, 0], robot_joints[2]),  # Elbow
                ([0, 0, 0.200], [1, 0, 0], robot_joints[3]),  # Wrist 1
                ([1.5925, 0, 0], [0, 1, 0], robot_joints[4]), # Wrist 2
                ([0.200, 0, 0], [1, 0, 0], robot_joints[5])   # Wrist 3
            ]
            
            for translation, axis, angle in transforms:
                T_joint = np.eye(4)
                T_joint[0:3, 3] = translation
                T_joint[0:3, 0:3] = self._axis_angle_to_rotation_matrix(np.array(axis), angle)
                T = T @ T_joint
            
            # Extract position and orientation
            position = T[0:3, 3]
            orientation = self._rotation_matrix_to_quaternion(T[0:3, 0:3])
            
            return position, orientation
            
        except Exception:
            return None, None
        finally:
            # Restore original state if changed
            if temp_positions is not None:
                self.current_joint_positions = temp_positions

    def _axis_angle_to_rotation_matrix(self, axis: np.ndarray, angle: float) -> np.ndarray:
        """Convert axis-angle to rotation matrix using Rodrigues' formula"""
        if np.linalg.norm(axis) == 0:
            return np.eye(3)
            
        axis = axis / np.linalg.norm(axis)
        K = np.array([[0, -axis[2], axis[1]],
                    [axis[2], 0, -axis[0]],
                    [-axis[1], axis[0], 0]])
        
        return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)

    def _rotation_matrix_to_quaternion(self, R: np.ndarray) -> np.ndarray:
        """Convert 3x3 rotation matrix to quaternion [qw, qx, qy, qz]"""
        trace = np.trace(R)
        
        if trace > 0:
            s = np.sqrt(trace + 1.0) * 2
            qw = 0.25 * s
            qx = (R[2, 1] - R[1, 2]) / s
            qy = (R[0, 2] - R[2, 0]) / s
            qz = (R[1, 0] - R[0, 1]) / s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s
        else:
            s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s
        
        return np.array([qw, qx, qy, qz])

    def get_current_pose(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Get current end-effector pose using current joint state"""
        if not self.state_valid:
            return None, None
        return self.forward_kinematics()