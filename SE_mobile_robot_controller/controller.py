#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
import math

class MyRobotNode(Node):
    def __init__(self):
        super().__init__('controller')
        self.get_logger().info('Hello World from SE_mobile_robot_controller!')

        # Goal coordinates (initialized to origin)
        self.goal_x = None
        self.goal_y = None

        # Controller gains
        self.Kp_yaw = 3.0  # Proportional gain for yaw control
        self.Kp_linear = 0.5  # Proportional gain for linear control

        # Publisher for cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for current pose
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            '/Pose',
            self.pose_callback,
            10
        )

        # Subscriber for goal pose
        self.goal_subscriber = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

    def goal_callback(self, msg):
        """Callback function for goal pose subscriber."""
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.get_logger().info(f'New goal received: x={self.goal_x:.3f}, y={self.goal_y:.3f}')

    def pose_callback(self, msg):
        """Callback function for pose subscriber."""
        # Extract position
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        # Extract quaternion orientation
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        # Convert quaternion to Euler angles (roll, pitch, yaw)
        # Using the standard quaternion to Euler conversion formulas
        roll = math.atan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))
        pitch = math.asin(2*(qw*qy - qz*qx))
        yaw = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))

        # Log pose and angles
        self.get_logger().info(f'Pose: x={x:.3f}, y={y:.3f}, z={z:.3f}')
        self.get_logger().info(f'Angles: roll={roll:.3f}, pitch={pitch:.3f}, yaw={yaw*180/math.pi:.3f}')
        
        if self.goal_x is not None and self.goal_y is not None:
            error_x = self.goal_x - x
            error_y = self.goal_y - y
            error_yaw = math.atan2(error_y, error_x) - yaw

            while(error_yaw > math.pi):
                error_yaw -= 2 * math.pi
            while(error_yaw < -math.pi):
                error_yaw += 2 * math.pi
            
            # Calculate distance to goal
            distance_error = math.sqrt(error_x**2 + error_y**2)

            self.get_logger().info(f'Error: x={error_x:.3f}, y={error_y:.3f}, distance={distance_error:.3f}, yaw={error_yaw*180/math.pi:.3f}')

            if distance_error < 0.02:
                return
            
            # Proportional controller for yaw
            angular_velocity = self.Kp_yaw * error_yaw

            # Proportional controller for linear velocity
            # Only move forward if roughly facing the goal (within 30 degrees)
            heading_tolerance = math.pi / 6  # 30 degrees
            if abs(error_yaw) < heading_tolerance:
                linear_velocity = self.Kp_linear * distance_error
                # Limit linear velocity
                max_linear_vel = 0.5  # m/s
                if distance_error > 0.01:
                    linear_velocity = min(max_linear_vel, linear_velocity)
                else:   
                    linear_velocity = 0
            else:
                linear_velocity = 0.0  # Don't move forward if not facing goal

            # Limit angular velocity to reasonable bounds
            max_angular_vel = 3.0  # rad/s
            angular_velocity = max(-max_angular_vel, min(max_angular_vel, angular_velocity))

            # Create and publish cmd_vel message
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = linear_velocity
            cmd_vel_msg.angular.z = angular_velocity
            self.cmd_vel_publisher.publish(cmd_vel_msg)

            self.get_logger().info(f'Command: linear_x={linear_velocity:.3f}, angular_z={angular_velocity:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = MyRobotNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
