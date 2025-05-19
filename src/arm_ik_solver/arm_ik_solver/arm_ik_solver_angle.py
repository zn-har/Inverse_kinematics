#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray
import math
import time


class subscriber(Node):
    def __init__(self):
         super().__init__('point_subscriber')
         self.subsriber=self.create_subscription(Point,'mytopic',self.logic,20)
         self.publisher = self.create_publisher(Float64MultiArray, '/arm_angles', 10)
         time.sleep(10.0)

    def logic(self,msg:Point):
         x=msg.x
         y=msg.y
         z=msg.z

         # Link lengths (example values in meters)
         L1 = 0.5  # Base to shoulder
         L2 = 0.3  # Shoulder to elbow
         L3 = 0.2  # Elbow to wrist

         def inverse_kinematics_5dof(x, y, z, pitch, roll):
            """
            Compute inverse kinematics for a simplified 5-DOF robotic arm.
            Inputs:
                x, y, z   - desired end effector position
                pitch     - desired pitch angle (rad)
                roll      - desired roll angle (rad)
            Returns:
                A tuple of 5 joint angles (theta1 to theta5)
            """

            # θ1 - base rotation angle
            theta1 = math.atan2(y, x)

            # Planar distance to wrist position
            r = math.sqrt(x**2 + y**2)
            z_wrist = z - L1  # subtract base height

            # Distance from shoulder to wrist
            D = math.sqrt(r**2 + z_wrist**2)

            # Law of Cosines for elbow angle (θ3)
            cos_theta3 = (D**2 - L2**2 - L3**2) / (2 * L2 * L3)
            if abs(cos_theta3) > 1.0:
                raise ValueError("Target is out of reach.")
            theta3 = math.acos(cos_theta3)

            # Angle from shoulder to wrist
            theta2_offset = math.atan2(z_wrist, r)
            theta2_inner = math.acos((L2**2 + D**2 - L3**2) / (2 * L2 * D))
            theta2 = theta2_offset + theta2_inner

            # θ4 and θ5 are wrist pitch and roll
            theta4 = pitch
            theta5 = roll

            return theta1, theta2, theta3, theta4, theta5

        # Example usage
        #if __name__ == "__main__":
        # Target position and orientation
         target_x = x
         target_y = y
         target_z = z
         pitch = math.radians(30)
         roll = math.radians(15)

         try:
            angles = list(inverse_kinematics_5dof(target_x, target_y, target_z, pitch, roll))
            self.get_logger().info("Joint angles (radians):"+str(angles))
            #self.get_logger().info("Joint angles (degrees):", tuple(math.degrees(a) for a in angles))


            # joint_state = JointState()
            # joint_state.header.stamp = self.get_clock().now().to_msg()
            # joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']  # match your URDF joints
            # joint_state.position = list(angles)

            # self.publisher.publish(joint_state)
            msg = Float64MultiArray()
            msg.data = angles
            self.publisher.publish(msg)
            self.get_logger().info(f'Published angles: {msg.data}')

         except ValueError as e:
            print("Error:", e)
def main(args=None):
    rclpy.init(args=args)
    node = subscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

        