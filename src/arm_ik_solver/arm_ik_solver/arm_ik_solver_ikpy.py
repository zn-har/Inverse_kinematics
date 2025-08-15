#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray

import numpy as np

# Try to import ikpy as fallback IK solver
try:
    import ikpy.chain
    import ikpy.link
    IKPY_AVAILABLE = True
except ImportError:
    IKPY_AVAILABLE = False

class IKSolverIKPyNode(Node):
    def __init__(self):
        super().__init__('ik_solver_ikpy_node')
        
        if not IKPY_AVAILABLE:
            self.get_logger().error('ikpy library not available. Install with: pip install ikpy')
            return

        # Create a simple 5-DOF arm chain using ikpy
        # You'll need to adjust these parameters based on your actual robot
        self.arm_chain = ikpy.chain.Chain.from_urdf_file(
            '/home/zenhar/hor/RoboticArmSim_Dockerized/workspace/src/arm_ik_solver/arm_ik_solver/five_dof_arm.urdf'
        ) if self.check_urdf_exists() else self.create_default_chain()

        # Publisher for joint angles
        self.publisher = self.create_publisher(
            Float64MultiArray,
            'arm_angles',
            10
        )
        
        # Subscriber for target positions
        self.subscriber = self.create_subscription(
            Point,
            'targets',
            self.target_callback,
            10
        )

        self.get_logger().info('IKPy IK Solver Node has started.')
        if IKPY_AVAILABLE:
            self.get_logger().info(f'Arm chain has {len(self.arm_chain.links)} links')

    def check_urdf_exists(self):
        """Check if URDF file exists"""
        import os
        urdf_path = '/home/zenhar/hor/RoboticArmSim_Dockerized/workspace/src/arm_ik_solver/arm_ik_solver/five_dof_arm.urdf'
        return os.path.exists(urdf_path)

    def create_default_chain(self):
        """Create a default 5-DOF arm chain if URDF is not available"""
        self.get_logger().warn('URDF not found, creating default arm configuration')
        
        # Define a simple 5-DOF arm with reasonable link lengths
        links = [
            ikpy.link.OriginLink(),  # Base link (fixed)
            ikpy.link.URDFLink(
                name="shoulder_pan",
                origin_translation=[0, 0, 0.1],
                origin_orientation=[0, 0, 0],
                rotation=[0, 0, 1],  # Rotate around Z axis
                bounds=(-np.pi, np.pi)
            ),
            ikpy.link.URDFLink(
                name="shoulder_lift",
                origin_translation=[0, 0, 0.2],
                origin_orientation=[0, 0, 0],
                rotation=[0, 1, 0],  # Rotate around Y axis
                bounds=(-np.pi/2, np.pi/2)
            ),
            ikpy.link.URDFLink(
                name="elbow",
                origin_translation=[0, 0, 0.2],
                origin_orientation=[0, 0, 0],
                rotation=[0, 1, 0],  # Rotate around Y axis
                bounds=(-np.pi, 0)
            ),
            ikpy.link.URDFLink(
                name="wrist_1",
                origin_translation=[0, 0, 0.15],
                origin_orientation=[0, 0, 0],
                rotation=[0, 1, 0],  # Rotate around Y axis
                bounds=(-np.pi, np.pi)
            ),
            ikpy.link.URDFLink(
                name="wrist_2",
                origin_translation=[0, 0, 0.1],
                origin_orientation=[0, 0, 0],
                rotation=[1, 0, 0],  # Rotate around X axis
                bounds=(-np.pi, np.pi)
            )
        ]
        
        return ikpy.chain.Chain(name='five_dof_arm', links=links)

    def target_callback(self, msg):
        if not IKPY_AVAILABLE:
            self.get_logger().error('ikpy not available for IK computation')
            return

        x = msg.x
        y = msg.y
        z = msg.z + 1  # Add offset to match the other solver

        target = [x, y, z]
        self.get_logger().info(f'Received target point: {target}')

        try:
            # Define target position (4x4 transformation matrix)
            target_position = np.array([
                [1, 0, 0, x],
                [0, 1, 0, y], 
                [0, 0, 1, z],
                [0, 0, 0, 1]
            ])

            # Compute inverse kinematics
            joint_angles = self.arm_chain.inverse_kinematics(target_position)
            
            # Remove the first element if it's from the OriginLink
            if len(joint_angles) > 5:
                joint_angles = joint_angles[1:]
            
            self.get_logger().info(f'IK solution found: {joint_angles}')

            # Publish joint angles
            angle_msg = Float64MultiArray()
            angle_msg.data = joint_angles.tolist()
            self.publisher.publish(angle_msg)
            
        except Exception as e:
            self.get_logger().error(f'IK computation failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = IKSolverIKPyNode()
    
    if IKPY_AVAILABLE:
        rclpy.spin(node)
    else:
        node.get_logger().error('Cannot run without ikpy library')
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
