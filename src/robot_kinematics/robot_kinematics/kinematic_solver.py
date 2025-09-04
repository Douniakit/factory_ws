"""Kinematic solver for the flange robot system"""

import numpy as np
from scipy.optimize import least_squares
from typing import Optional
from enum import Enum

class IKMethod(Enum):
    """Available IK solution methods"""
    DAMPED_LEAST_SQUARES = "damped_least_squares"
    LEVENBERG_MARQUARDT = "levenberg_marquardt"

class KinematicSolver:
    """Kinematic solver for the flange robot system"""
    
    def __init__(self, robot_kinematics):
        self.robot = robot_kinematics
        self.config = {
            'max_iterations': 100,
            'tolerance': 1e-6,
            'step_size': 0.5,
            'damping_factor': 0.01
        }
    
    def solve_ik(self, target_position: np.ndarray, target_orientation: np.ndarray,
                 initial_guess: Optional[np.ndarray] = None,
                 method: IKMethod = IKMethod.DAMPED_LEAST_SQUARES) -> Optional[np.ndarray]:
        """Solve inverse kinematics"""
        
        if initial_guess is None:
            initial_guess = np.zeros(7)
        
        target_orientation = self._normalize_quaternion(target_orientation)
        
        if method == IKMethod.DAMPED_LEAST_SQUARES:
            return self._solve_damped_least_squares(target_position, target_orientation, initial_guess)
        elif method == IKMethod.LEVENBERG_MARQUARDT:
            return self._solve_levenberg_marquardt(target_position, target_orientation, initial_guess)
        
        return None
    
    def _solve_damped_least_squares(self, target_pos, target_ori, q0):
        """Damped Least Squares method"""
        q = q0.copy()
        
        for iteration in range(self.config['max_iterations']):
            pos, ori = self.robot.forward_kinematics(q)
            if pos is None or ori is None:
                return None
            
            error = self._compute_task_error(target_pos, target_ori, pos, ori)
            error_norm = np.linalg.norm(error)
            
            if error_norm < self.config['tolerance']:
                return q
            
            J = self._compute_jacobian(q)
            if J is None:
                return None
            
            # FIX: Use the pseudo-inverse approach for rectangular Jacobian
            # J is 6x7 (6 task dimensions, 7 joint dimensions)
            damping = self.config['damping_factor']
            
            # Method 1: Using the damped pseudo-inverse (recommended)
            JtJ = J.T @ J  # This is 7x7
            Jt = J.T       # This is 7x6
            dq = Jt @ np.linalg.solve(J @ Jt + damping * np.eye(6), error)  # Solve 6x6 system
            
            dq *= self.config['step_size']
            q = q + dq
        
        return None
    
    def _solve_levenberg_marquardt(self, target_pos, target_ori, q0):
        """Levenberg-Marquardt method using scipy"""
        def residual_func(q):
            pos, ori = self.robot.forward_kinematics(q)
            if pos is None or ori is None:
                return np.ones(6) * 1e6
            return self._compute_task_error(target_pos, target_ori, pos, ori)
        
        result = least_squares(
            residual_func,
            q0,
            method='trf',
            ftol=self.config['tolerance'],
            max_nfev=self.config['max_iterations'] * 10
        )
        
        return result.x if result.success else None
    
    def _compute_jacobian(self, q: np.ndarray) -> Optional[np.ndarray]:
        """Compute Jacobian using finite differences"""
        n_joints = len(q)
        J = np.zeros((6, n_joints))
        
        pos0, ori0 = self.robot.forward_kinematics(q)
        if pos0 is None or ori0 is None:
            return None
        
        delta = 1e-6
        
        for i in range(n_joints):
            q_plus = q.copy()
            q_plus[i] += delta
            
            pos_plus, ori_plus = self.robot.forward_kinematics(q_plus)
            if pos_plus is None or ori_plus is None:
                return None
            
            J[:3, i] = (pos_plus - pos0) / delta
            J[3:, i] = self._quaternion_error(ori_plus, ori0) / delta
        
        return J
    
    def _compute_task_error(self, target_pos, target_ori, current_pos, current_ori):
        """Compute 6D task space error"""
        pos_error = target_pos - current_pos
        ori_error = self._quaternion_error(target_ori, current_ori) * 0.5
        return np.concatenate([pos_error, ori_error])
    
    def _normalize_quaternion(self, q):
        """Normalize quaternion to unit length"""
        q = np.array(q)
        norm = np.linalg.norm(q)
        if norm < 1e-6:
            return np.array([1.0, 0.0, 0.0, 0.0])
        return q / norm
    
    def _quaternion_error(self, q_target, q_current):
        """Compute quaternion error as axis-angle vector"""
        q_target = self._normalize_quaternion(q_target)
        q_current = self._normalize_quaternion(q_current)
        
        # Error quaternion
        q_current_inv = np.array([q_current[0], -q_current[1], -q_current[2], -q_current[3]])
        
        w1, v1 = q_target[0], q_target[1:]
        w2, v2 = q_current_inv[0], q_current_inv[1:]
        
        w_error = w1 * w2 - np.dot(v1, v2)
        v_error = w1 * v2 + w2 * v1 + np.cross(v1, v2)
        q_error = np.array([w_error, v_error[0], v_error[1], v_error[2]])
        
        if q_error[0] < 0:
            q_error = -q_error
        
        if abs(q_error[0]) > 0.9999:
            return np.zeros(3)
        
        angle = 2 * np.arccos(np.clip(q_error[0], -1, 1))
        sin_half_angle = np.sqrt(1 - q_error[0]**2)
        
        if sin_half_angle < 1e-6:
            return 2 * q_error[1:]
        
        axis = q_error[1:] / sin_half_angle
        return axis * angle