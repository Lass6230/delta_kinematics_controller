#!/usr/bin/env python3
"""
Script to publish test forces to the mock FT sensor for admittance controller testing.

Usage:
  # Apply 5N force in X direction
  ros2 run delta_kinematics_controller publish_test_force.py --fx 5.0

  # Apply force in multiple directions
  ros2 run delta_kinematics_controller publish_test_force.py --fx 2.0 --fy 1.0 --fz -3.0

  # Apply torque
  ros2 run delta_kinematics_controller publish_test_force.py --tx 0.5 --ty 0.2
"""

import argparse
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import SetHardwareComponentState
from std_msgs.msg import Float64MultiArray
import time


class ForceTorquePublisher(Node):
    def __init__(self):
        super().__init__('ft_publisher')
        
        # Publisher for FT sensor commands
        # The mock hardware with mock_sensor_commands=true will listen to this
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/tcp_fts_sensor/commands',
            10
        )
        
        self.get_logger().info('FT Sensor Command Publisher initialized')
    
    def publish_force_torque(self, fx=0.0, fy=0.0, fz=0.0, tx=0.0, ty=0.0, tz=0.0):
        """
        Publish force-torque command to the mock sensor.
        
        Args:
            fx, fy, fz: Forces in X, Y, Z (Newtons)
            tx, ty, tz: Torques around X, Y, Z (Newton-meters)
        """
        msg = Float64MultiArray()
        # Order matches the state interfaces: force.x, force.y, force.z, torque.x, torque.y, torque.z
        msg.data = [fx, fy, fz, tx, ty, tz]
        
        self.publisher.publish(msg)
        self.get_logger().info(
            f'Published F=[{fx:.2f}, {fy:.2f}, {fz:.2f}] N, '
            f'T=[{tx:.2f}, {ty:.2f}, {tz:.2f}] Nm'
        )


def main():
    parser = argparse.ArgumentParser(
        description='Publish test forces to mock FT sensor for admittance controller testing'
    )
    parser.add_argument('--fx', type=float, default=0.0, help='Force in X direction (N)')
    parser.add_argument('--fy', type=float, default=0.0, help='Force in Y direction (N)')
    parser.add_argument('--fz', type=float, default=0.0, help='Force in Z direction (N)')
    parser.add_argument('--tx', type=float, default=0.0, help='Torque around X axis (Nm)')
    parser.add_argument('--ty', type=float, default=0.0, help='Torque around Y axis (Nm)')
    parser.add_argument('--tz', type=float, default=0.0, help='Torque around Z axis (Nm)')
    parser.add_argument('--rate', type=float, default=10.0, help='Publishing rate (Hz)')
    parser.add_argument('--duration', type=float, default=5.0, help='Duration to publish (seconds), 0 for continuous')
    
    args = parser.parse_args()
    
    rclpy.init()
    node = ForceTorquePublisher()
    
    rate = node.create_rate(args.rate)
    start_time = time.time()
    
    try:
        node.get_logger().info(
            f'Publishing force/torque at {args.rate} Hz for {args.duration if args.duration > 0 else "infinite"} seconds...'
        )
        node.get_logger().info('Press Ctrl+C to stop')
        
        while rclpy.ok():
            node.publish_force_torque(
                fx=args.fx, fy=args.fy, fz=args.fz,
                tx=args.tx, ty=args.ty, tz=args.tz
            )
            
            # Check duration
            if args.duration > 0 and (time.time() - start_time) > args.duration:
                node.get_logger().info('Duration reached, stopping...')
                break
            
            rate.sleep()
            
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        # Send zero force before exiting
        node.get_logger().info('Sending zero force/torque...')
        for _ in range(5):
            node.publish_force_torque(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            time.sleep(0.1)
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
