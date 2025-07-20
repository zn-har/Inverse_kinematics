#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


class Mynode(Node):
    def __init__(self):
        super().__init__("kinematics")
        #self.get_logger().info("hellloooo")
        self.publisher=self.create_publisher(Point,'targets',20)
        self.a=0
        self.create_timer(1.0,self.logic)

    def logic(self):
        try:
            x = float(input("Enter x coordinate: "))
            y = float(input("Enter y coordinate: "))
            z = float(input("Enter z coordinate: "))
        except ValueError:
            self.get_logger().info("Invalid input. Please enter numeric values.")
            return
        point = Point()
        point.x = x
        point.y = y
        point.z = z+1

        self.publisher.publish(point)

def main(args=None):
    rclpy.init(args=args)
    node=Mynode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()
