#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray
import math


class subscriber(Node):
    def __init__(self):
        super().__init__('point_subscriber')
        self.subscriber = self.create_subscription(Point, 'mytopic', self.logic, 20)
        self.publisher = self.create_publisher(Float64MultiArray, '/arm_angles', 10)

    def inverse_kinematics_5dof(self, x, y, z, pitch, roll):
        """
        Compute inverse kinematics for a 5-DOF robotic arm.
        
        With only 5 DOF, the arm cannot reach arbitrary position and orientation.
        This implementation prioritizes position and approximates orientation.
        
        Args:
            x, y, z: Target end effector position (meters)
            pitch: Desired pitch angle (radians)
            roll: Desired roll angle (radians)
        
        Returns:
            Tuple of 5 joint angles (radians)
        """
        # Corrected link lengths based on model.sdf
        L1 = 1.0    # Base to shoulder (Z offset)
        L2 = 0.43   # Upper arm length
        L3 = 0.4    # Elbow link length
        L4 = 0.036  # Forearm length
        L5 = 0.035  # End effector offset
        
        # Joint 1: Base rotation
        theta1 = math.atan2(y, x)
        
        # Calculate position in plane after base rotation
        r = math.sqrt(x**2 + y**2)  # Distance in XY plane
        z_shoulder = z - L1  # Height relative to shoulder
        
        # Account for wrist and end effector length in target calculation
        # This is a simplification - assumes end effector pointing outward
        wrist_r = r
        wrist_z = z_shoulder
        
        # Distance from shoulder to wrist
        D = math.sqrt(wrist_r**2 + wrist_z**2)
        
        # Check if target is reachable
        max_reach = L2 + L3 + L4 + L5
        if D > max_reach:
            raise ValueError(f"Target out of reach. Distance to target ({D:.2f}m) exceeds arm length ({max_reach:.2f}m)")
        
        # Joint 3: Elbow angle
        cos_theta3 = (D**2 - L2**2 - L3**2) / (2 * L2 * L3)
        cos_theta3 = max(min(cos_theta3, 1.0), -1.0)  # Clamp to valid range
        theta3 = -math.acos(cos_theta3)  # Negative for more natural pose
        
        # Joint 2: Shoulder angle
        beta = math.atan2(wrist_z, wrist_r)  # Angle to target
        cos_alpha = (L2**2 + D**2 - L3**2) / (2 * L2 * D)
        cos_alpha = max(min(cos_alpha, 1.0), -1.0)  # Clamp to valid range
        alpha = math.acos(cos_alpha)  # Angle within arm triangle
        theta2 = beta + alpha
        
        # Joint 4: Wrist rotation (around Z axis)
        # For 5-DOF, orientation control is limited
        theta4 = roll
        
        # Joint 5: Wrist pitch
        # Compensate for arm's orientation to approximate desired pitch
        arm_pitch = theta2 + theta3  # Combined pitch from shoulder and elbow
        theta5 = pitch - arm_pitch
        
        # Check joint limits
        angles = [theta1, theta2, theta3, theta4, theta5]
        limits = [
            (-3.14, 3.14),   # joint1
            (-0.34, 1.57),   # joint2
            (-1.74, 1.74),   # joint3
            (-3.14, 3.14),   # joint4
            (-1.27, 1.27)    # joint5
        ]
        
        for i, (angle, (lower, upper)) in enumerate(zip(angles, limits)):
            if angle < lower:
                self.get_logger().warning(f"Joint {i+1} angle {angle:.2f} below limit, clamping to {lower:.2f}")
                angles[i] = lower
            elif angle > upper:
                self.get_logger().warning(f"Joint {i+1} angle {angle:.2f} above limit, clamping to {upper:.2f}")
                angles[i] = upper
        
        return tuple(angles)

    def logic(self, msg: Point):
        x = msg.x
        y = msg.y
        z = msg.z

        # Default orientation (you can later receive from a topic)
        pitch = math.radians(30)
        roll = math.radians(15)

        try:
            angles = list(self.inverse_kinematics_5dof(x, y, z, pitch, roll))
            self.get_logger().info("Joint angles (radians): " + str(angles))

            msg_out = Float64MultiArray()
            msg_out.data = angles
            self.publisher.publish(msg_out)
            self.get_logger().info(f'Published angles: {msg_out.data}')

        except ValueError as e:
            self.get_logger().error(f"IK Error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = subscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted.')
    finally:
        node.destroy_node()
        rclpy.shutdown()
