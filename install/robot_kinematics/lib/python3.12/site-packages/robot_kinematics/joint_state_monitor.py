
# ROS2 script that provides a ROS2 node to monitor and log joint states for a 7-DOF robotic system

"""A ROS2 node to monitor and log joint states for a 7-DOF robotic system"""


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np

class JointStateMonitor(Node):
    def __init__(self):
        super().__init__('joint_state_monitor')
        
        # Subscribe to joint states
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Expected joints for your 7-DOF system (based on URDF analysis)
        self.expected_joints = [
            'joint1',           # Linear axis (prismatic) 
            'joint_1',          # Robot joint 1 (revolute)
            'joint_2',          # Robot joint 2 (revolute)
            'joint_3',          # Robot joint 3 (revolute)
            'joint_4',          # Robot joint 4 (revolute)
            'joint_5',          # Robot joint 5 (revolute)
            'joint_6',          # Robot joint 6 (revolute)
        ]
        
        self.get_logger().info('Joint State Monitor started - waiting for joint states...')
        
    def joint_state_callback(self, msg: JointState):
        """Process incoming joint state messages"""
        
        # Display raw message info
        self.get_logger().info(f"Received joint states:")
        self.get_logger().info(f"  Number of joints: {len(msg.name)}")
        self.get_logger().info(f"  Joint names: {msg.name}")
        
        # Check if we have the expected joints
        missing_joints = []
        available_joints = {}
        
        for expected_joint in self.expected_joints:
            if expected_joint in msg.name:
                idx = msg.name.index(expected_joint)
                position = msg.position[idx] if idx < len(msg.position) else None
                velocity = msg.velocity[idx] if idx < len(msg.velocity) else None
                effort = msg.effort[idx] if idx < len(msg.effort) else None
                available_joints[expected_joint] = {
                    'position': position,
                    'velocity': velocity,
                    'effort': effort,
                    'index': idx
                }
            else:
                missing_joints.append(expected_joint)
        
        # Report findings
        if missing_joints:
            self.get_logger().warn(f"Missing expected joints: {missing_joints}")
        else:
            self.get_logger().info(" All expected joints found!")
        
        # Display joint information in kinematic chain order
        self.get_logger().info("=== 7-DOF KINEMATIC CHAIN ===")
        joint_positions = []
        
        for i, joint_name in enumerate(self.expected_joints):
            if joint_name in available_joints:
                joint_info = available_joints[joint_name]
                joint_type = "PRISMATIC" if joint_name == "joint1" else "REVOLUTE"
                position = joint_info['position']
                velocity = joint_info['velocity']
                effort = joint_info['effort']
                
                # Format position and velocity based on joint type
                if joint_type == "PRISMATIC":
                    pos_str = f"{position:.4f} m" if position is not None else "N/A"
                    vel_str = f"{velocity:.4f} m/s" if velocity is not None else "N/A"
                    eff_str = f"{effort:.2f} N" if effort is not None else "N/A"
                else:
                    pos_deg = np.degrees(position) if position is not None else None
                    vel_deg = np.degrees(velocity) if velocity is not None else None
                    pos_str = f"{position:.4f} rad ({pos_deg:.2f}°)" if position is not None else "N/A"
                    vel_str = f"{velocity:.4f} rad/s ({vel_deg:.2f}°/s)" if velocity is not None else "N/A"
                    eff_str = f"{effort:.2f} Nm" if effort is not None else "N/A"
                
                self.get_logger().info(f"  DOF{i+1}: {joint_name} ({joint_type})")
                self.get_logger().info(f"    Position: {pos_str}")
                self.get_logger().info(f"    Velocity: {vel_str}")
                self.get_logger().info(f"    Effort:   {eff_str}")
                
                # Add to joint positions vector
                if position is not None:
                    joint_positions.append(position)
                    
            else:
                self.get_logger().error(f"  DOF{i+1}: {joint_name} - MISSING!")
        
        # Create joint position vector for kinematics calculations
        if len(joint_positions) == 7:
            joint_array = np.array(joint_positions)
            self.get_logger().info("=" * 50)
            self.get_logger().info(" READY FOR KINEMATICS CALCULATIONS!")
            self.get_logger().info(f"Complete 7-DOF joint vector (for IK/FK):")
            self.get_logger().info(f"q = {joint_array}")
            self.get_logger().info(f"Linear axis (m):     {joint_array[0]:.4f}")
            self.get_logger().info(f"Robot joints (rad):  {joint_array[1:]}")
            self.get_logger().info(f"Robot joints (deg):  {np.degrees(joint_array[1:])}")
            
            # Additional info for kinematics
            self.log_kinematics_info(joint_array)
            
        else:
            self.get_logger().warn(f" Incomplete joint data - got {len(joint_positions)}/7 joints")
            self.get_logger().warn("Cannot perform kinematics calculations!")
        
        self.get_logger().info("=" * 70)

    def log_kinematics_info(self, joint_vector):
        """Log additional information useful for kinematics calculations"""
        
        linear_pos = joint_vector[0]
        robot_joints = joint_vector[1:]
        
        # Linear axis range check
        linear_range = (-0.41, 7.3)  # From URDF
        if linear_range[0] <= linear_pos <= linear_range[1]:
            linear_status = "✅ Within range"
        else:
            linear_status = "⚠️  OUT OF RANGE!"
        
        self.get_logger().info(f"Linear axis: {linear_pos:.4f}m {linear_status} (range: {linear_range[0]} to {linear_range[1]}m)")
        
        # Robot joint range checks (approximate from URDF)
        robot_ranges = [
            (-170, 170),   # joint_1: ±170°
            (-65, 85),     # joint_2: -65° to +85°
            (-180, 70),    # joint_3: -180° to +70°
            (-300, 300),   # joint_4: ±300°
            (-130, 130),   # joint_5: ±130°
            (-360, 360),   # joint_6: ±360°
        ]
        
        for i, (angle_rad, (min_deg, max_deg)) in enumerate(zip(robot_joints, robot_ranges)):
            angle_deg = np.degrees(angle_rad)
            if min_deg <= angle_deg <= max_deg:
                status = "✅"
            else:
                status = "⚠️ "
            self.get_logger().info(f"Robot joint_{i+1}: {angle_deg:.2f}° {status} (range: {min_deg}° to {max_deg}°)")


def main(args=None):
    rclpy.init(args=args)
    joint_monitor = JointStateMonitor()
    
    try:
        rclpy.spin(joint_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        joint_monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()