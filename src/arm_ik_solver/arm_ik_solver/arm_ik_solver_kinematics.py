#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
from ikpy.chain import Chain
import numpy as np
import math
import os
from ament_index_python.packages import get_package_share_directory

class IKSolverNode(Node):
    def __init__(self):
        super().__init__('ik_solver_node')

        # Load URDF
        urdf_path = os.path.join(
            get_package_share_directory('arm_ik_solver'),
            'urdf',
            'five_dof_arm.urdf'
        )

        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"URDF not found at {urdf_path}")

        self.get_logger().info(f"Loading URDF: {urdf_path}")
        self.chain = Chain.from_urdf_file(urdf_path, active_links_mask=[False] + [True]*5)
        self.get_logger().info(f"Chain loaded with {len(self.chain.links)} links.")
        
        self.joint_limits = [
            link.bounds for link in self.chain.links[1:]
            if link.bounds is not None
        ]
        
        # Tolerance for FK verification
        self.error_tolerance = 0.05

        # ROS setup
        self.publisher = self.create_publisher(Float64MultiArray, 'arm_angles', 10)
        self.subscriber = self.create_subscription(Pose, 'targets', self.target_callback, 10)

    def quaternion_to_euler(self, qx, qy, qz, qw):
        """Convert quaternion to euler angles (roll, pitch, yaw)"""
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        sinp = 2 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
            
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw

    def target_callback(self, msg):
        target_position = [msg.position.x, msg.position.y, msg.position.z]
        
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w
        
        roll, pitch, yaw = self.quaternion_to_euler(qx, qy, qz, qw)
        
        self.get_logger().info(f"Target position: {target_position}")
        self.get_logger().info(f"Target orientation - Roll: {math.degrees(roll):.1f}°, Pitch: {math.degrees(pitch):.1f}°, Yaw: {math.degrees(yaw):.1f}°")

        try:
            # STEP 1: Solve IK for POSITION ONLY (ignore orientation)
            # This gets us to the X,Y,Z location
            ik_solution = self.chain.inverse_kinematics(target_position)

            if len(ik_solution) != 6:
                raise ValueError("Invalid IK output length.")

            # Get the position-based joint angles
            position_angles = list(ik_solution[1:])  # Skip base link
            
            # STEP 2: Modify ONLY the end-effector orientation joints
            # Based on your arm structure:
            # - Joint 1: Shoulder rotation (affects position)
            # - Joint 2: Shoulder pitch (affects position)  
            # - Joint 3: Elbow pitch (affects position)
            # - Joint 4: Forearm twist (orientation)
            # - Joint 5: Wrist pitch (orientation)
            
            # Keep joints 1-3 from position IK (for reaching X,Y,Z)
            final_angles = position_angles[:3]
            
            # Set orientation joints directly from input
            forearm_twist = yaw  # Map yaw to joint 4 (forearm twist)
            wrist_pitch = pitch  # Map pitch to joint 5 (wrist pitch)
            
            final_angles.append(forearm_twist)  # Joint 4
            final_angles.append(wrist_pitch)    # Joint 5
            
            # Joint limit checking and clamping
            for i, (angle, (low, high)) in enumerate(zip(final_angles, self.joint_limits)):
                if not (low <= angle <= high):
                    self.get_logger().warn(f"Joint {i+1} angle {angle:.3f} rad out of limits [{low:.3f}, {high:.3f}]")
                    final_angles[i] = max(low, min(high, angle))
                    self.get_logger().info(f"Clamped joint {i+1} to {final_angles[i]:.3f} rad")

            # Verify the position is still reachable (orientation might have affected it)
            full_solution = [0] + final_angles
            fk_transform = self.chain.forward_kinematics(full_solution)
            fk_position = fk_transform[:3, 3]
            position_error = np.linalg.norm(np.array(target_position) - fk_position)
            
            if position_error > self.error_tolerance:
                self.get_logger().warn(f"Position error after orientation adjustment: {position_error:.3f} m")
                # Still publish - the position might be close enough
            
            # Print results
            print(f"\n--- TWO-STEP CONTROL ---")
            print(f"Step 1 - Position IK angles: {[round(a, 3) for a in position_angles]}")
            print(f"Step 2 - Added orientation:")
            print(f"  Joint 4 (forearm twist): {math.degrees(forearm_twist):.1f}° (from yaw)")
            print(f"  Joint 5 (wrist pitch): {math.degrees(wrist_pitch):.1f}° (from pitch)")
            print(f"Final joint angles: {[round(a, 3) for a in final_angles]}")

            msg_out = Float64MultiArray()
            msg_out.data = final_angles
            self.publisher.publish(msg_out)

        except Exception as e:
            self.get_logger().error(f"IK computation failed: {e}")
            self.publish_invalid()

    def publish_invalid(self):
        msg_out = Float64MultiArray()
        msg_out.data = [float('nan')] * 5
        self.publisher.publish(msg_out)
        self.get_logger().warn("Published invalid solution (NaN angles)")

def main(args=None):
    rclpy.init(args=args)
    node = IKSolverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
