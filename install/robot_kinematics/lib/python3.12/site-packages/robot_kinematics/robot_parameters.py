# ROS2 script that provides the robot parameters class

"""Robot parameters extracted directly from your URDF files"""  


import numpy as np
from dataclasses import dataclass
from typing import Dict, List, Optional

@dataclass
class JointInfo:
    """Joint information extracted from URDF"""
    name: str
    joint_type: str  # 'revolute' or 'prismatic'
    origin: np.ndarray  # [x, y, z]
    rpy: np.ndarray     # [roll, pitch, yaw] 
    axis: np.ndarray    # [x, y, z]
    limits: Dict        # {'lower': float, 'upper': float, 'velocity': float, 'effort': float}
    parent_link: str
    child_link: str

class RobotParameters:
    """Robot parameters extracted directly from your URDF files"""
    
    def __init__(self):
        self.n_dof = 7
        
        # Extract joint information from your URDFs
        self.joints = self._load_urdf_joint_data()
        
        # Custom kinematic parameters from ABB robot tag
        self.custom_params = self._load_custom_parameters()
        
        # Linear axis to robot connection (from your launch file)
        self.linear_axis_to_robot_transform = self._get_connection_transform()
        
        # Joint names in kinematic chain order
        self.joint_names = ['joint1', 'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        
    def _load_urdf_joint_data(self) -> Dict[str, JointInfo]:
        """Load joint data exactly from your URDF files"""
        
        joints = {}
        
        # Linear axis joint (from linear_axis.urdf)
        joints['joint1'] = JointInfo(
            name='joint1',
            joint_type='prismatic',
            origin=np.array([0.0, 0.0, 0.0]),
            rpy=np.array([0.0, 0.0, 0.0]),
            axis=np.array([1.0, 0.0, 0.0]),  # X-axis movement
            limits={
                'lower': -0.41,
                'upper': 7.3,
                'velocity': 1.0,
                'effort': 1000.0
            },
            parent_link='base_link',
            child_link='link_1'
        )
        
        # Robot joints (from abb_irb6700_150_320.urdf)
        joints['joint_1'] = JointInfo(
            name='joint_1',
            joint_type='revolute',
            origin=np.array([0.0, 0.0, 0.780]),  # xyz="0 0 0.780"
            rpy=np.array([0.0, 0.0, 0.0]),       # rpy="0 0 0"
            axis=np.array([0.0, 0.0, 1.0]),      # axis="0 0 1"
            limits={
                'lower': -2.9670597283903604,
                'upper': 2.9670597283903604, 
                'velocity': 1.7453292519943295,
                'effort': None  # Not specified in URDF
            },
            parent_link='base_link',
            child_link='link_1'
        )
        
        joints['joint_2'] = JointInfo(
            name='joint_2',
            joint_type='revolute',
            origin=np.array([0.320, 0.0, 0.0]),  # xyz="0.320 0 0"
            rpy=np.array([0.0, 0.0, 0.0]),
            axis=np.array([0.0, 1.0, 0.0]),      # axis="0 1 0"
            limits={
                'lower': -1.1344640137963142,
                'upper': 1.4835298641951802,
                'velocity': 1.5707963267948966,
                'effort': None
            },
            parent_link='link_1',
            child_link='link_2'
        )
        
        joints['joint_3'] = JointInfo(
            name='joint_3',
            joint_type='revolute',
            origin=np.array([0.0, 0.0, 1.280]),  # xyz="0 0 1.280"
            rpy=np.array([0.0, 0.0, 0.0]),
            axis=np.array([0.0, 1.0, 0.0]),      # axis="0 1 0"
            limits={
                'lower': -3.141592653589793,
                'upper': 1.2217304763960306,
                'velocity': 1.5707963267948966,
                'effort': None
            },
            parent_link='link_2',
            child_link='link_3'
        )
        
        joints['joint_4'] = JointInfo(
            name='joint_4',
            joint_type='revolute',
            origin=np.array([0.0, 0.0, 0.200]),  # xyz="0 0 0.200"
            rpy=np.array([0.0, 0.0, 0.0]),
            axis=np.array([1.0, 0.0, 0.0]),      # axis="1 0 0"
            limits={
                'lower': -5.235987755982989,
                'upper': 5.235987755982989,
                'velocity': 2.9670597283903604,
                'effort': None
            },
            parent_link='link_3',
            child_link='link_4'
        )
        
        joints['joint_5'] = JointInfo(
            name='joint_5',
            joint_type='revolute',
            origin=np.array([1.5925, 0.0, 0.0]), # xyz="1.5925 0 0"
            rpy=np.array([0.0, 0.0, 0.0]),
            axis=np.array([0.0, 1.0, 0.0]),      # axis="0 1 0"
            limits={
                'lower': -2.2689280275926285,
                'upper': 2.2689280275926285,
                'velocity': 2.0943951023931953,
                'effort': None
            },
            parent_link='link_4',
            child_link='link_5'
        )
        
        joints['joint_6'] = JointInfo(
            name='joint_6',
            joint_type='revolute',
            origin=np.array([0.200, 0.0, 0.0]),  # xyz="0.200 0 0"
            rpy=np.array([0.0, 0.0, 0.0]),
            axis=np.array([1.0, 0.0, 0.0]),      # axis="1 0 0"
            limits={
                'lower': -6.283185307179586,
                'upper': 6.283185307179586,
                'velocity': 3.3161255787892263,
                'effort': None
            },
            parent_link='link_5',
            child_link='link_6'
        )
        
        return joints
    
    def _load_custom_parameters(self) -> Dict[str, float]:
        """Custom kinematic parameters from your ABB robot tag"""
        return {
            'a1': 0.320,
            'a2': -0.200,
            'b': 0.0,
            'c1': 0.780,
            'c2': 1.280,
            'c3': 1.5925,
            'c4': 0.200,
            'acceleration_factor': 2,
            'jerk_factor': 64,
            'control_rate': 250,
            'offsets2': -np.pi / 2
        }
    
    def _get_connection_transform(self) -> Dict:
        """Connection between linear axis and robot from launch file"""
        return {
            'linear_axis_flange_to_link1': np.array([0.6, 0.43, 0.55]),  # From linear axis URDF
            'robot_mounting_offset': np.array([0.0, 0.0, 0.1]),         # From launch file
            'total_offset': np.array([0.6, 0.43, 0.65])  # Combined
        }
    
    def get_joint_limits(self, joint_name: str) -> Dict:
        """Get limits for a specific joint"""
        if joint_name in self.joints:
            return self.joints[joint_name].limits
        return None
    
    def is_joint_within_limits(self, joint_name: str, position: float) -> bool:
        """Check if joint position is within URDF limits"""
        limits = self.get_joint_limits(joint_name)
        if limits is None:
            return False
        return limits['lower'] <= position <= limits['upper']
    
    def get_transformation_matrix(self, joint_name: str, joint_angle: float) -> np.ndarray:
        """
        Get 4x4 transformation matrix for a joint using URDF data
        This avoids DH parameter conversion - uses direct URDF transformations
        """
        if joint_name not in self.joints:
            raise ValueError(f"Joint {joint_name} not found")
            
        joint_info = self.joints[joint_name]
        
        # Translation from URDF origin
        translation = joint_info.origin
        
        # Rotation from URDF rpy (fixed part)
        rpy = joint_info.rpy
        R_fixed = self._rpy_to_rotation_matrix(rpy[0], rpy[1], rpy[2])
        
        # Joint rotation (variable part)
        if joint_info.joint_type == 'revolute':
            R_joint = self._axis_angle_to_rotation_matrix(joint_info.axis, joint_angle)
        elif joint_info.joint_type == 'prismatic':
            R_joint = np.eye(3)
            # For prismatic joint, add to translation along axis
            translation = translation + joint_angle * joint_info.axis
        else:
            R_joint = np.eye(3)
        
        # Combined rotation
        R_total = R_fixed @ R_joint
        
        # Build 4x4 transformation matrix
        T = np.eye(4)
        T[0:3, 0:3] = R_total
        T[0:3, 3] = translation
        
        return T
    
    def _rpy_to_rotation_matrix(self, roll: float, pitch: float, yaw: float) -> np.ndarray:
        """Convert roll-pitch-yaw to rotation matrix"""
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        
        R = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp,   cp*sr,            cp*cr           ]
        ])
        
        return R
    
    def _axis_angle_to_rotation_matrix(self, axis: np.ndarray, angle: float) -> np.ndarray:
        """Convert axis-angle to rotation matrix (Rodrigues' formula)"""
        axis = axis / np.linalg.norm(axis)  # Normalize
        K = np.array([[0, -axis[2], axis[1]],
                      [axis[2], 0, -axis[0]],
                      [-axis[1], axis[0], 0]])
        
        R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)
        return R
    
    def print_summary(self):
        """Print a summary of the robot parameters"""
        print("=== ROBOT PARAMETERS (FROM URDF) ===")
        print(f"Total DOF: {self.n_dof}")
        print(f"Joint chain: {' -> '.join(self.joint_names)}")
        print("\nJoint Details:")
        
        for joint_name in self.joint_names:
            joint = self.joints[joint_name]
            limits = joint.limits
            print(f"  {joint_name} ({joint.joint_type}):")
            print(f"    Origin: {joint.origin}")
            print(f"    Axis: {joint.axis}")
            if joint.joint_type == 'prismatic':
                print(f"    Limits: {limits['lower']:.3f} to {limits['upper']:.3f} m")
            else:
                print(f"    Limits: {np.degrees(limits['lower']):.1f}° to {np.degrees(limits['upper']):.1f}°")