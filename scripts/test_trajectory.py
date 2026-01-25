#!/usr/bin/env python3
"""
Test script to send a simple joint trajectory to the delta robot.
This will command the joint_trajectory_controller while the delta_kinematics_controller
monitors the state and publishes forward kinematics.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class TrajectoryTester(Node):
    def __init__(self):
        super().__init__('trajectory_tester')
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )
        self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()
        self.get_logger().info('Action server available!')

    def send_simple_trajectory(self):
        """Send a simple sinusoidal trajectory"""
        goal_msg = FollowJointTrajectory.Goal()
        
        # Create trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = ['joint1', 'joint2', 'joint3']
        
        # Define waypoints (small movements around zero position)
        waypoints = [
            ([0.0, 0.0, 0.0], 0.0),      # Start position
            ([0.1, 0.0, -0.1], 2.0),     # Move joints 1 and 3
            ([0.0, 0.1, 0.0], 4.0),      # Move joint 2
            ([-0.1, 0.0, 0.1], 6.0),     # Move joints 1 and 3 opposite
            ([0.0, -0.1, 0.0], 8.0),     # Move joint 2 opposite
            ([0.0, 0.0, 0.0], 10.0),     # Return to start
        ]
        
        for positions, time_sec in waypoints:
            point = JointTrajectoryPoint()
            point.positions = positions
            point.velocities = [0.0, 0.0, 0.0]
            point.time_from_start = Duration(sec=int(time_sec), nanosec=int((time_sec % 1) * 1e9))
            trajectory.points.append(point)
        
        goal_msg.trajectory = trajectory
        
        self.get_logger().info('Sending trajectory with {} waypoints...'.format(len(trajectory.points)))
        
        # Send goal
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return False
        
        self.get_logger().info('Goal accepted! Executing trajectory...')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        self.get_logger().info('Trajectory execution finished with error code: {}'.format(result.result.error_code))
        
        return result.result.error_code == 0


def main(args=None):
    rclpy.init(args=args)
    
    tester = TrajectoryTester()
    
    try:
        # Send trajectory
        success = tester.send_simple_trajectory()
        
        if success:
            tester.get_logger().info('✓ Trajectory executed successfully!')
            tester.get_logger().info('Check the /ee_pose topic to see forward kinematics updates')
            tester.get_logger().info('Check the TF tree to see base->ee transform')
        else:
            tester.get_logger().error('✗ Trajectory execution failed!')
    
    except KeyboardInterrupt:
        tester.get_logger().info('Interrupted by user')
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
