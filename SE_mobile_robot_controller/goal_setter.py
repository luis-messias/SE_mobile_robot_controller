#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import argparse
import sys
import time

class GoalSetterNode(Node):
    """A ROS2 node to set goal positions via command line."""

    def __init__(self, goal_x, goal_y):
        super().__init__('goal_setter')

        # Publisher for goal_pose
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # Create and publish goal pose multiple times to ensure delivery
        for i in range(3):
            self.publish_goal(goal_x, goal_y)
            time.sleep(0.1)

        self.get_logger().info(f'Goal set to ({goal_x}, {goal_y})')

    def publish_goal(self, x, y):
        """Publish a goal pose message."""
        msg = PoseStamped()

        # Set header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        # Set position
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0

        # Set orientation (facing positive x direction)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        # Publish the message
        self.goal_publisher.publish(msg)
        self.get_logger().info(f'Published goal pose: x={x}, y={y}')


def main(args=None):
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Set goal position for robot controller')
    parser.add_argument('x', type=float, help='Goal X coordinate')
    parser.add_argument('y', type=float, help='Goal Y coordinate')

    # Parse arguments
    parsed_args = parser.parse_args()

    # Initialize ROS2
    rclpy.init(args=args)

    # Create and run the node
    node = GoalSetterNode(parsed_args.x, parsed_args.y)

    # Keep the node alive to ensure message is published and processed
    try:
        # Spin for a short time to allow message publishing
        rclpy.spin_once(node, timeout_sec=0.1)
        # Add a small delay to ensure message is sent
        time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()