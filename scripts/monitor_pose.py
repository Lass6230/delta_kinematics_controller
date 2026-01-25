#!/usr/bin/env python3
"""
Monitor the end-effector pose from the delta kinematics controller
and display it in a readable format.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math


class PoseMonitor(Node):
    def __init__(self):
        super().__init__('pose_monitor')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/ee_pose',
            self.pose_callback,
            10
        )
        self.get_logger().info('Monitoring /ee_pose topic...')
        self.get_logger().info('Position units: METERS')
        self.get_logger().info('---')

    def pose_callback(self, msg):
        pos = msg.pose.position
        ori = msg.pose.orientation
        
        # Convert quaternion to euler angles (yaw only for simplicity)
        yaw = math.atan2(2.0 * (ori.w * ori.z + ori.x * ori.y),
                        1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z))
        
        self.get_logger().info(
            f'EE Position [m]: x={pos.x:+.4f}, y={pos.y:+.4f}, z={pos.z:+.4f} | '
            f'yaw={math.degrees(yaw):+.2f}°'
        )


def main(args=None):
    rclpy.init(args=args)
    monitor = PoseMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        monitor.get_logger().info('Shutting down...')
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
