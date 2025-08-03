#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import math


class Mynode(Node):
    def __init__(self):
        super().__init__("kinematics")
        #self.get_logger().info("hellloooo")
        self.publisher=self.create_publisher(Pose,'targets',20)
        self.a=0
        self.create_timer(1.0,self.logic)

    def logic(self):
        try:
            x = float(input("Enter x coordinate: "))
            y = float(input("Enter y coordinate: "))
            z = float(input("Enter z coordinate: "))
            roll = float(input("Enter roll (degrees): "))
            pitch = float(input("Enter pitch (degrees): "))
            yaw = float(input("Enter yaw (degrees): "))
        except ValueError:
            self.get_logger().info("Invalid input. Please enter numeric values.")
            return
        
        # Convert degrees to radians
        roll_rad = math.radians(roll)
        pitch_rad = math.radians(pitch)
        yaw_rad = math.radians(yaw)
        
        # Convert Euler angles to quaternion (simple ZYX convention)
        cy = math.cos(yaw_rad * 0.5)
        sy = math.sin(yaw_rad * 0.5)
        cp = math.cos(pitch_rad * 0.5)
        sp = math.sin(pitch_rad * 0.5)
        cr = math.cos(roll_rad * 0.5)
        sr = math.sin(roll_rad * 0.5)
        
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z+1
        
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        self.publisher.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    node=Mynode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()
