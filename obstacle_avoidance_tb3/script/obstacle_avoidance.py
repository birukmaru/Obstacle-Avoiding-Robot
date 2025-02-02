#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.laser_scan_callback, 10)
        self.timer = self.create_timer(0.1, self.publish_velocity)  # Faster control loop

        self.velocity_msg = Twist()
        self.max_forward_speed = 0.4
        self.forward_speed = self.max_forward_speed
        self.turn_speed = 0.0
        self.obstacle_threshold = 0.6  # Meters

        # Controller parameters
        self.kp = 1.0  # Proportional gain for turning
        self.recovery_mode = False
        self.recovery_count = 0

    def laser_scan_callback(self, msg):
        # Focus on the front 60 degrees (330 to 30 degrees)
        field_range = 60
        initial_angle = 330
        obstacle_detected = False

        min_distance = float('inf')
        for i in range(initial_angle, initial_angle + field_range):
            distance = msg.ranges[i % 360]
            if distance < min_distance:
                min_distance = distance

            if distance < self.obstacle_threshold:
                obstacle_detected = True

        if obstacle_detected:
            self.recovery_mode = False  # Reset recovery if actively avoiding
            self.forward_speed = 0.0

            # Proportional control for turning
            error = self.obstacle_threshold - min_distance
            self.turn_speed = self.kp * error
            self.turn_speed = max(min(self.turn_speed, 0.6), -0.6)  # Limit turn speed
        else:
            self.forward_speed = self.max_forward_speed
            self.turn_speed = 0.0

        # Recovery behavior if stuck
        if self.recovery_count > 20:
            self.initiate_recovery()

        if obstacle_detected:
            self.recovery_count += 1
        else:
            self.recovery_count = 0

    def initiate_recovery(self):
        self.get_logger().warn("Initiating recovery behavior...")
        self.recovery_mode = True
        self.forward_speed = -0.2  # Move backward
        self.turn_speed = 0.5      # Rotate
        self.recovery_count = 0

    def publish_velocity(self):
        self.velocity_msg.linear.x = self.forward_speed
        self.velocity_msg.angular.z = self.turn_speed
        self.publisher.publish(self.velocity_msg)

    def stop_robot(self):
        self.velocity_msg.linear.x = 0.0
        self.velocity_msg.angular.z = 0.0
        self.publisher.publish(self.velocity_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("Shutting down obstacle avoidance...")
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

