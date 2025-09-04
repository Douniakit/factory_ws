# ROS2 node that provides kinematics services
# subscribes to joint states, publishes current end-effector pose, target pose, and IK solutions

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import numpy as np

from .robot_kinematics import RobotKinematics
from .kinematic_solver import KinematicSolver, IKMethod

class KinematicsServiceNode(Node):
    def __init__(self):
        super().__init__('robot_kinematics_service')
        
        # Initialize robot and solver
        self.robot = RobotKinematics(logger=self.get_logger())
        self.solver = KinematicSolver(self.robot)
        self.current_joint_positions = None
        
        # Subscribers
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
       
        # Publishers 
        self.current_pose_pub = self.create_publisher(PoseStamped, '/robot_kinematics/current_pose', 10) # current end-effector pose
        self.target_pose_pub = self.create_publisher(PoseStamped, '/robot_kinematics/target_pose', 10) # desired target pose
        self.ik_joint_state_pub = self.create_publisher(JointState, '/robot_kinematics/ik_joint_states', 10)
        
        # Timers
        self.create_timer(0.1, self.publish_current_pose)
        self.create_timer(1.0, self.publish_target_pose)  # Publish target pose every second
        self.create_timer(2.0, self.calculate_and_publish_ik)  # Calculate IK every 2 seconds
        
        self.get_logger().info("Kinematics Service Node initialized")
    
    def joint_state_callback(self, msg: JointState):
        joint_positions = []
        for joint_name in self.robot.params.joint_names:
            if joint_name in msg.name:
                idx = msg.name.index(joint_name)
                joint_positions.append(msg.position[idx] if idx < len(msg.position) else 0.0)
            else:
                joint_positions.append(0.0)
        
        self.current_joint_positions = np.array(joint_positions)
        self.robot.update_joint_state(
            self.current_joint_positions,
            np.zeros(len(joint_positions)),
            np.zeros(len(joint_positions))
        )
    
    # NEW METHOD: This is where the IK calculation happens
    def calculate_and_publish_ik(self):
        """Calculate IK for target pose and publish the joint solution"""
        if self.current_joint_positions is None:
            self.get_logger().warn("No joint states received yet - cannot calculate IK")
            return
        
        # Define target pose (hardcoded for now)
        target_position = np.array([2.768321593846597, 0.43, 1.9826680935984418])
        target_orientation = np.array([0.9747904092118523, 0.0, 0.22312251815670578, 0.0])
        
        self.get_logger().info("Calculating IK solution...")
        
        # Solve IK using current joint positions as initial guess
        ik_solution = self.solver.solve_ik(
            target_position,
            target_orientation,
            initial_guess=self.current_joint_positions,
            method=IKMethod.LEVENBERG_MARQUARDT
        )
        
        if ik_solution is not None:
            self.get_logger().info(" IK solution found!")
            
            # Verify the solution by computing forward kinematics
            fk_pos, fk_ori = self.robot.forward_kinematics(ik_solution)
            if fk_pos is not None:
                pos_error = np.linalg.norm(target_position - fk_pos)
                # Compute and log error
                self.get_logger().info(f"Position error: {pos_error:.6f} m")
                # print the solution
                self.get_logger().info(f"IK Joint Positions: {ik_solution}")
            
            # Publish the IK solution
            self.publish_ik_solution(ik_solution, 'base_link')
        else:
            self.get_logger().error(" IK solution failed!")
    
    def publish_ik_solution(self, ik_solution: np.ndarray, frame_id: str):
        ik_msg = JointState()
        ik_msg.header.stamp = self.get_clock().now().to_msg()
        ik_msg.header.frame_id = frame_id
        ik_msg.name = self.robot.params.joint_names
        ik_msg.position = ik_solution.tolist()
        self.ik_joint_state_pub.publish(ik_msg)
        self.get_logger().info("Published IK solution")
    
    def publish_current_pose(self):
        position, orientation = self.robot.get_current_pose()
        if position is not None and orientation is not None:
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'base_link'
            
            pose_msg.pose.position.x = float(position[0])
            pose_msg.pose.position.y = float(position[1])
            pose_msg.pose.position.z = float(position[2])
            
            pose_msg.pose.orientation.w = float(orientation[0])
            pose_msg.pose.orientation.x = float(orientation[1])
            pose_msg.pose.orientation.y = float(orientation[2])
            pose_msg.pose.orientation.z = float(orientation[3])
            
            self.current_pose_pub.publish(pose_msg)  # FIXED: Use correct publisher

    def publish_target_pose(self):
        target_position = np.array([2.768321593846597, 0.43, 1.9826680935984418])
        target_orientation = np.array([0.9747904092118523, 0.0, 0.22312251815670578, 0.0])

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'base_link'
        
        pose_msg.pose.position.x = float(target_position[0])
        pose_msg.pose.position.y = float(target_position[1])
        pose_msg.pose.position.z = float(target_position[2])
        
        pose_msg.pose.orientation.w = float(target_orientation[0])
        pose_msg.pose.orientation.x = float(target_orientation[1])
        pose_msg.pose.orientation.y = float(target_orientation[2])
        pose_msg.pose.orientation.z = float(target_orientation[3])
        
        self.target_pose_pub.publish(pose_msg)  # FIXED: Use correct publisher

def main(args=None):
    rclpy.init(args=args)
    try:
        node = KinematicsServiceNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()