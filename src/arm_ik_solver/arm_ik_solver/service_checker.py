#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
import sys

class ServiceChecker(Node):
    def __init__(self):
        super().__init__('service_checker')
        
        self.get_logger().info('Checking available ROS 2 services...')
        
        try:
            # Get list of available services
            result = subprocess.run(['ros2', 'service', 'list'], 
                                  capture_output=True, text=True, timeout=10)
            
            if result.returncode == 0:
                services = result.stdout.strip().split('\n')
                self.get_logger().info(f'Found {len(services)} services:')
                
                # Look for MoveIt-related services
                moveit_services = [s for s in services if any(keyword in s.lower() 
                                 for keyword in ['moveit', 'ik', 'plan', 'compute'])]
                
                if moveit_services:
                    self.get_logger().info('MoveIt-related services found:')
                    for service in moveit_services:
                        self.get_logger().info(f'  - {service}')
                else:
                    self.get_logger().warn('No MoveIt-related services found!')
                    self.get_logger().info('All available services:')
                    for service in services:
                        self.get_logger().info(f'  - {service}')
                        
            else:
                self.get_logger().error(f'Failed to get service list: {result.stderr}')
                
        except subprocess.TimeoutExpired:
            self.get_logger().error('Timeout while checking services')
        except Exception as e:
            self.get_logger().error(f'Error checking services: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    node = ServiceChecker()
    
    # Keep the node alive for a short time to see the output
    import time
    time.sleep(2)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
