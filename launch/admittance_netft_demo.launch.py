#!/usr/bin/env python3
# Copyright 2024
# Licensed under the Apache License, Version 2.0

"""
Launch file for Delta robot with Network FT Sensor and Admittance Controller.

This demo uses a real network-based force-torque sensor (ATI, OnRobot, etc.)
via the net_ft_driver package to enable admittance control.

Controller Chain:
  Hardware -> Joint Trajectory Controller -> Admittance Controller -> User Commands
               (receives velocity refs)        (exports position/velocity refs)
  
  FT Sensor (network) -> Force Torque Sensor Broadcaster -> Admittance Controller
                         (publishes /ft_data)               (reads tcp_fts_sensor)
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
            description='Use fake hardware (true) or real hardware with network FT sensor (false)',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'ft_sensor_ip',
            default_value='192.168.1.1',
            description='IP address of the network FT sensor',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'ft_sensor_type',
            default_value='ati',
            description='Type of FT sensor: ati, ati_axia, or onrobot',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'ft_rdt_sampling_rate',
            default_value='1000',
            description='RDT sampling rate in Hz',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'ft_filter_rate',
            default_value='100',
            description='Internal filter rate in Hz',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'ft_use_hardware_biasing',
            default_value='true',
            description='Use hardware biasing for FT sensor',
        )
    )

    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    ft_sensor_ip = LaunchConfiguration('ft_sensor_ip')
    ft_sensor_type = LaunchConfiguration('ft_sensor_type')
    ft_rdt_sampling_rate = LaunchConfiguration('ft_rdt_sampling_rate')
    ft_filter_rate = LaunchConfiguration('ft_filter_rate')
    ft_use_hardware_biasing = LaunchConfiguration('ft_use_hardware_biasing')

    # Get delta robot with Net FT sensor URDF
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("delta_kinematics_controller"),
                    "urdf",
                    "delta_with_netft.urdf.xacro",
                ]
            ),
            " use_fake_hardware:=",
            use_fake_hardware,
            " use_netft:=true",
            " ft_sensor_ip:=",
            ft_sensor_ip,
            " ft_sensor_type:=",
            ft_sensor_type,
            " ft_rdt_sampling_rate:=",
            ft_rdt_sampling_rate,
            " ft_filter_rate:=",
            ft_filter_rate,
            " ft_use_hardware_biasing:=",
            ft_use_hardware_biasing,
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # Controller configuration for admittance with Net FT sensor
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("delta_kinematics_controller"),
            "config",
            "admittance_netft_controllers.yaml",
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

    # 2. Delta kinematics controller (MUST be before admittance for plugin loading)
    delta_kinematics_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["my_delta_controller", "-c", "/controller_manager"],
    )

    # 3. Joint trajectory controller (must be loaded before admittance controller)
    #    This receives commands from the admittance controller
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    # 4. Force-Torque sensor broadcaster (publishes /ft_data from network sensor)
    force_torque_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["force_torque_sensor_broadcaster", "-c", "/controller_manager"],
    )

    # 5. Admittance controller (chained on top of joint trajectory controller)
    #    This exports reference interfaces that feed into joint_trajectory_controller
    #    Reads force data from tcp_fts_sensor (provided by force_torque_sensor_broadcaster)
    admittance_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "admittance_controller",
            "-c", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
    )

    # 6. Net FT diagnostic broadcaster (monitors sensor health)
    net_ft_diagnostic_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["net_ft_diagnostic_broadcaster", "-c", "/controller_manager"],
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delta_kinematics_controller_spawner,
        joint_trajectory_controller_spawner,
        force_torque_sensor_broadcaster_spawner,
        admittance_controller_spawner,
        net_ft_diagnostic_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
