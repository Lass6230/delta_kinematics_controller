#!/usr/bin/env python3
"""
Test script for Admittance Controller Demo

This script helps verify that the admittance controller chain is working correctly.
It sends test commands and monitors the system response.

Usage:
    python3 test_admittance_demo.py
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from control_msgs.msg import AdmittanceControllerState
import time
import sys


class AdmittanceDemoTester(Node):
    def __init__(self):
        super().__init__('admittance_demo_tester')
        
        # Publishers
        self.joint_ref_pub = self.create_publisher(
            JointTrajectoryPoint,
            '/admittance_controller/joint_references',
            10
        )
        
        self.wrench_ref_pub = self.create_publisher(
            WrenchStamped,
            '/admittance_controller/wrench_reference',
            10
        )
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.admittance_state_sub = self.create_subscription(
            AdmittanceControllerState,
            '/admittance_controller/state',
            self.admittance_state_callback,
            10
        )
        
        self.joint_states_received = False
        self.admittance_states_received = False
        self.latest_joint_state = None
        self.latest_admittance_state = None
        
        self.get_logger().info('Admittance Demo Tester initialized')
        
    def joint_state_callback(self, msg):
        self.joint_states_received = True
        self.latest_joint_state = msg
        
    def admittance_state_callback(self, msg):
        self.admittance_states_received = True
        self.latest_admittance_state = msg
    
    def wait_for_topics(self, timeout=5.0):
        """Wait for topics to be available"""
        self.get_logger().info('Waiting for topics...')
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            if self.joint_states_received and self.admittance_states_received:
                self.get_logger().info('✓ All topics available')
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not self.joint_states_received:
            self.get_logger().error('✗ /joint_states not available')
        if not self.admittance_states_received:
            self.get_logger().error('✗ /admittance_controller/state not available')
        
        return False
    
    def send_position_command(self, positions):
        """Send position command to admittance controller"""
        msg = JointTrajectoryPoint()
        msg.positions = positions
        msg.velocities = [0.0] * len(positions)
        msg.accelerations = [0.0] * len(positions)
        
        self.joint_ref_pub.publish(msg)
        self.get_logger().info(f'Sent position command: {positions}')
    
    def send_wrench_command(self, force, torque):
        """Send wrench command to admittance controller"""
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'end_effector'
        msg.wrench.force.x = force[0]
        msg.wrench.force.y = force[1]
        msg.wrench.force.z = force[2]
        msg.wrench.torque.x = torque[0]
        msg.wrench.torque.y = torque[1]
        msg.wrench.torque.z = torque[2]
        
        self.wrench_ref_pub.publish(msg)
        self.get_logger().info(f'Sent wrench: Force={force}, Torque={torque}')
    
    def print_current_state(self):
        """Print current robot state"""
        if self.latest_joint_state:
            self.get_logger().info('--- Current Joint State ---')
            for name, pos in zip(self.latest_joint_state.name, 
                                self.latest_joint_state.position):
                self.get_logger().info(f'  {name}: {pos:.4f} rad')
        
        if self.latest_admittance_state:
            self.get_logger().info('--- Admittance Controller State ---')
            self.get_logger().info(f'  Wrench: Force=[{self.latest_admittance_state.wrench.force.x:.2f}, '
                                 f'{self.latest_admittance_state.wrench.force.y:.2f}, '
                                 f'{self.latest_admittance_state.wrench.force.z:.2f}]')


def run_test_sequence(tester):
    """Run a sequence of test commands"""
    logger = tester.get_logger()
    
    # Wait for system to be ready
    if not tester.wait_for_topics():
        logger.error('System not ready. Make sure the admittance demo is running.')
        return False
    
    logger.info('\n========================================')
    logger.info('Starting Admittance Controller Test')
    logger.info('========================================\n')
    
    # Test 1: Print initial state
    logger.info('TEST 1: Checking initial state')
    time.sleep(1.0)
    rclpy.spin_once(tester, timeout_sec=0.1)
    tester.print_current_state()
    time.sleep(2.0)
    
    # Test 2: Send position command
    logger.info('\nTEST 2: Sending position command [0.1, 0.0, 0.0]')
    tester.send_position_command([0.1, 0.0, 0.0])
    time.sleep(3.0)
    rclpy.spin_once(tester, timeout_sec=0.1)
    tester.print_current_state()
    
    # Test 3: Return to zero
    logger.info('\nTEST 3: Returning to zero position')
    tester.send_position_command([0.0, 0.0, 0.0])
    time.sleep(3.0)
    rclpy.spin_once(tester, timeout_sec=0.1)
    tester.print_current_state()
    
    # Test 4: Send wrench command
    logger.info('\nTEST 4: Sending wrench command (5N in Z direction)')
    for _ in range(30):  # Send for 3 seconds at 10Hz
        tester.send_wrench_command([0.0, 0.0, 5.0], [0.0, 0.0, 0.0])
        time.sleep(0.1)
        rclpy.spin_once(tester, timeout_sec=0.01)
    
    tester.print_current_state()
    
    # Test 5: Stop wrench
    logger.info('\nTEST 5: Stopping wrench command')
    tester.send_wrench_command([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    time.sleep(2.0)
    rclpy.spin_once(tester, timeout_sec=0.1)
    tester.print_current_state()
    
    logger.info('\n========================================')
    logger.info('Test sequence completed!')
    logger.info('========================================\n')
    
    return True


def main(args=None):
    rclpy.init(args=args)
    
    tester = AdmittanceDemoTester()
    
    try:
        success = run_test_sequence(tester)
        
        if success:
            tester.get_logger().info('All tests passed! ✓')
            return 0
        else:
            tester.get_logger().error('Tests failed! ✗')
            return 1
            
    except KeyboardInterrupt:
        tester.get_logger().info('Test interrupted by user')
        return 0
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
