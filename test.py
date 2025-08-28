#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time

class DWATestNode(Node):
    def __init__(self):
        super().__init__('dwa_test_node')
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.get_logger().info("DWA Test Node started")
        
    def send_goal(self, x, y, theta=0.0):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.position.z = 0.0
        
        # Convert theta to quaternion
        # goal.pose.orientation.x = 0.0
        # goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = theta
        goal.pose.orientation.w = 1.0
        
        self.goal_pub.publish(goal)
        self.get_logger().info(f"Sent goal: ({x}, {y})")

def main():
    rclpy.init()
    test_node = DWATestNode()
    
    # Wait for the planner to start
    time.sleep(2.0)
    
    # Send test goals
    test_goals = [
        # (0.0, -0.5),    # Forward
        (1.0, -0.5),    # Forward and left
        (2.0, 0.0),    # Left turn
        (-1.0, -0.5),   # Reverse
        (0.0, -0.5)     # Return to origin
    ]
    
    for x, y in test_goals:
        test_node.send_goal(x, y)
        test_node.get_logger().info(f"Goal sent: ({x}, {y}). Press Enter to send next goal...")
        input()  # Wait for user input
    
    test_node.get_logger().info("All test goals sent!")
    rclpy.shutdown()

if __name__ == '__main__':
    main()