#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class SafeTeleop(Node):
    def __init__(self):
        super().__init__('safe_teleop')
        self.teleop_sub = self.create_subscription(
            Twist, 'teleop_cmd_vel', self.teleop_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.safe_to_move = True
        self.current_cmd = Twist()

    def teleop_callback(self, msg):
        self.current_cmd = msg
        self.publish_safe_cmd()

    def scan_callback(self, scan_msg):
        min_distance = min(scan_msg.ranges)
        self.safe_to_move = min_distance >= 0.5

        if not self.safe_to_move:
            self.get_logger().warn(f"Obstacle detected! Distance: {min_distance:.2f}m")

        self.publish_safe_cmd()

    def publish_safe_cmd(self):
        cmd = Twist()
        if self.safe_to_move:
            cmd = self.current_cmd
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().warn("Blocking command - obstacle too close!")

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SafeTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down safely...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

