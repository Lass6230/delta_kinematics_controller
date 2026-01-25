#!/usr/bin/env python3
# Copyright 2024
# Licensed under the Apache License, Version 2.0

"""
Launch file for Delta robot with Admittance Controller demo.

This demo shows how to use:
1. Delta Kinematics Controller - for forward/inverse kinematics
2. Admittance Controller - for force-compliant behavior (chained)
3. Joint Trajectory Controller - for trajectory execution (receives from admittance controller)

The admittance controller provides a compliant interface that reacts to forces
measured by a force-torque sensor, making the robot compliant and safe for
human interaction or contact-rich tasks.

Controller Chain:
  Hardware -> Joint Trajectory Controller -> Admittance Controller -> User Commands
               (receives velocity refs)        (exports position/velocity refs)
"""

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue


def generate_launch_description():

    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='true',
            description='Start with fake hardware (set to false for real hardware)',
        )
    )

    use_fake_hardware = LaunchConfiguration('use_fake_hardware')

    # Get delta robot URDF for admittance demo
    # This uses mock joints but real FT sensor hardware
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("delta_kinematics_controller"),
                    "urdf",
                    "delta_admittance_demo.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # Controller configuration for admittance demo
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("delta_kinematics_controller"),
            "config",
            "admittance_demo_controllers.yaml",
        ]
    )

    # Nodes
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
        arguments=['--ros-args', '--log-level', 'WARN'],
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Spawn controllers in correct order for chaining
    # 1. Joint state broadcaster (always first)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    # 2. Delta kinematics controller (for lower joint states)
    delta_kinematics_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["my_delta_controller", "-c", "/controller_manager",
                   "--ros-args", "--log-level", "WARN"],
    )

    # 3. Force-torque sensor broadcaster (required by admittance controller)
    force_torque_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ft_sensor_broadcaster", "-c", "/controller_manager"],
    )

    # 4. Admittance controller (must be loaded FIRST in the chain)
    #    This claims the hardware command interfaces and exports reference interfaces
    admittance_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "admittance_controller",
            "-c", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
    )

    # 5. Joint trajectory controller (receives reference commands from admittance controller)
    #    This subscribes to the reference interfaces exported by admittance_controller
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delta_kinematics_controller_spawner,
        force_torque_sensor_broadcaster_spawner,
        admittance_controller_spawner,
        # joint_trajectory_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
