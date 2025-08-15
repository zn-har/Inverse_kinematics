#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Float64MultiArray
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState

import numpy as np

class IKSolverNode(Node):
    def __init__(self):
        super().__init__('ik_solver_node')

        # MoveIt configuration
        self.move_group_name = "arm"  # Replace with your actual move group name
        self.base_frame = "base_link"  # Replace with your actual base frame
        self.end_effector_link = "end_effector_link"  # Replace with your actual end effector link
        
        # Service client for IK - try multiple possible service names
        self.ik_service_names = [
            '/compute_ik',
            '/plan_kinematic_path',
            '/move_group/plan_kinematic_path'
        ]
        
        self.ik_client = None
        self.service_available = False
        
        # Try to connect to available IK services
        for service_name in self.ik_service_names:
            self.get_logger().info(f'Trying IK service: {service_name}')
            temp_client = self.create_client(GetPositionIK, service_name)
            
            # Wait for service with timeout
            if temp_client.wait_for_service(timeout_sec=2.0):
                self.ik_client = temp_client
                self.ik_service_name = service_name
                self.service_available = True
                self.get_logger().info(f'Connected to IK service: {service_name}')
                break
            else:
                self.get_logger().warn(f'Service {service_name} not available')
                temp_client.destroy()
        
        if not self.service_available:
            self.get_logger().error('No MoveIt IK service found! Make sure MoveIt is running.')
            self.get_logger().error('Available services can be checked with: ros2 service list')
            # Still continue to allow the node to run, but IK will fail

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
        z = msg.z + 1

        target = [x, y, z]
        self.get_logger().info(f'Received target point: {target}')

        if not self.service_available:
            self.get_logger().error('IK service not available. Cannot compute inverse kinematics.')
            return

        # Create IK request
        try:
            # Create pose for target position
            target_pose = PoseStamped()
            target_pose.header.frame_id = self.base_frame
            target_pose.header.stamp = self.get_clock().now().to_msg()
            target_pose.pose.position.x = x
            target_pose.pose.position.y = y
            target_pose.pose.position.z = z
            
            # Set default orientation (pointing down)
            target_pose.pose.orientation.x = 0.0
            target_pose.pose.orientation.y = 0.0
            target_pose.pose.orientation.z = 0.0
            target_pose.pose.orientation.w = 1.0

            # Create IK service request
            ik_request = GetPositionIK.Request()
            ik_request.ik_request.group_name = self.move_group_name
            ik_request.ik_request.robot_state.joint_state.header.stamp = self.get_clock().now().to_msg()
            ik_request.ik_request.robot_state.joint_state.header.frame_id = self.base_frame
            ik_request.ik_request.avoid_collisions = True
            ik_request.ik_request.pose_stamped = target_pose
            ik_request.ik_request.timeout.sec = 5
            ik_request.ik_request.timeout.nanosec = 0

            # Call IK service asynchronously
            future = self.ik_client.call_async(ik_request)
            future.add_done_callback(self.ik_response_callback)
            
        except Exception as e:
            self.get_logger().error(f'IK request creation failed: {e}')

    def ik_response_callback(self, future):
        """Handle the IK service response asynchronously"""
        try:
            ik_response = future.result()
            
            if ik_response.error_code.val == ik_response.error_code.SUCCESS:
                # Extract joint angles from the solution
                joint_state = ik_response.solution.joint_state
                joint_names = joint_state.name
                joint_positions = joint_state.position
                
                self.get_logger().info(f'IK solution found with {len(joint_positions)} joints')
                self.get_logger().info(f'Joint names: {joint_names}')
                self.get_logger().info(f'Joint angles: {joint_positions}')

                # Publish joint angles
                angle_msg = Float64MultiArray()
                angle_msg.data = list(joint_positions)
                self.publisher.publish(angle_msg)
                
            else:
                self.get_logger().error(f'IK solution failed with error code: {ik_response.error_code.val}')
                
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

