#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray
import math


class Subscriber(Node):
    def __init__(self):
        super().__init__('point_subscriber')
        self.subscriber = self.create_subscription(Point, 'mytopic', self.logic, 20)
        self.publisher = self.create_publisher(Float64MultiArray, '/arm_angles', 10)

    def inverse_kinematics_5dof(self, x, y, z, pitch, roll):
        L1 = 1.0
        L2 = 0.43
        L3 = 0.4
        L4 = 0.036
        L5 = 0.035

        theta1 = math.atan2(y, x)
        r = math.sqrt(x**2 + y**2)
        z_shoulder = z - L1
        wrist_r = r
        wrist_z = z_shoulder
        D = math.sqrt(wrist_r**2 + wrist_z**2)
        max_reach = L2 + L3 + L4 + L5

        if D > max_reach:
            raise ValueError(f"Target out of reach. Distance {D:.2f}m > max {max_reach:.2f}m")

        cos_theta3 = (D**2 - L2**2 - L3**2) / (2 * L2 * L3)
        cos_theta3 = max(min(cos_theta3, 1.0), -1.0)
        theta3 = -math.acos(cos_theta3)

        beta = math.atan2(wrist_z, wrist_r)
        cos_alpha = (L2**2 + D**2 - L3**2) / (2 * L2 * D)
        cos_alpha = max(min(cos_alpha, 1.0), -1.0)
        alpha = math.acos(cos_alpha)
        theta2 = beta + alpha - math.pi / 2

        theta4 = roll
        arm_pitch = theta2 + theta3
        theta5 = pitch - arm_pitch

        angles = [theta1, theta2, theta3, theta4, theta5]
        limits = [
            (-3.14, 3.14),
            (-0.34, 1.57),
            (-1.74, 1.74),
            (-3.14, 3.14),
            (-1.27, 1.27)
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

        try:
            pitch_deg = float(input("Enter pitch angle in degrees: "))
            roll_deg = float(input("Enter roll angle in degrees: "))
        except ValueError:
            self.get_logger().warn("Invalid input. Using default pitch=30° and roll=15°.")
            pitch_deg = 30.0
            roll_deg = 15.0

        pitch = math.radians(pitch_deg)
        roll = math.radians(roll_deg)

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
    node = Subscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted.')
    finally:
        node.destroy_node()
        rclpy.shutdown()
