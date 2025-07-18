#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray
from ikpy.chain import Chain

import os

class IKSolverNode(Node):
    def __init__(self):
        super().__init__('ik_solver_node')

        # Load URDF
        package_share = get_package_share_directory('arm_ik_solver')
        urdf_path = os.path.join(package_share, 'urdf', 'five_dof_arm.urdf')

        self.get_logger().info(f'Loading URDF from: {urdf_path}')

        try:
            if not os.path.exists(urdf_path):
                raise FileNotFoundError(f"URDF file not found at {urdf_path}")
            
            self.chain = Chain.from_urdf_file(urdf_path)
            self.get_logger().info(f'Successfully loaded chain with {len(self.chain.links)} links')
        except Exception as e:
            self.get_logger().error(f'Failed to load URDF: {e}')
            raise

        # Publisher for joint angles
        self.publisher = self.create_publisher(
            Float64MultiArray,
            'arm_angles',
            10
        )
        self.subscriber = self.create_subscription(
            Point,
            'targets',
            self.target_callback,
            10
        )

        self.get_logger().info('IK Solver Node has started.')

    def target_callback(self, msg):
        x = msg.x
        y = msg.y
        z = msg.z

        target = [x, y, z]
        self.get_logger().info(f'Received target point: {target}')

        # Compute IK
        try:
            ik_solution = self.chain.inverse_kinematics(target)
            
            if len(ik_solution) < 2:
                raise ValueError(f"IK solution has insufficient joints: {len(ik_solution)}")
                
            # ik_solution is a list of angles; skip first element (fixed base)
            angles = ik_solution[1:]

            self.get_logger().info(f'Calculated joint angles: {angles}')

            # Publish as Float64MultiArray
            angle_msg = Float64MultiArray()
            angle_msg.data = angles.tolist()
            self.publisher.publish(angle_msg)

        except Exception as e:
            self.get_logger().error(f'IK computation failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = IKSolverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

