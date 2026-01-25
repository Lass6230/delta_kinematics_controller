"""Launch demo ros2_control controller manager and spawn the delta controller.

This launch expects ros2_control and controller_manager packages available in your ROS 2
environment. It demonstrates how to configure the controller with parameters and spawn it.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition, UnlessCondition
import launch_ros.actions
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    pkg = 'delta_kinematics_controller'
    this_launch_dir = os.path.dirname(__file__)
    # Resolve config files relative to the installed share layout (this launch
    # file is installed into share/<pkg>/launch). This avoids calling
    # get_package_share_directory() during launch generation which can fail in
    # some runtime environments.
    default_yaml = os.path.join(this_launch_dir, '..', 'config', 'demo_controllers.yaml')
    
    ld = LaunchDescription()


    

    # Robot state publisher for visualization: generate robot_description from xacro
    # Resolve the xacro path relative to this launch file so the launch works even
    # when ament package discovery isn't available in the environment that
    # executes the xacro command. This uses the installed package layout when
    # available (the launch file is installed into share/<pkg>/launch).
    this_launch_dir = os.path.dirname(__file__)
    xacro_file = os.path.join(this_launch_dir, '..', 'urdf', 'delta_robot.urdf.xacro')
    robot_state_pub = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
    # The xacro contains reasonable defaults for geometry parameters. Omitting
    # explicit property overrides avoids xacro "redefining global" warnings
    # (which are printed on stderr and cause ros2 launch to treat the run as
    # a failure). If you need to override these values, update the xacro or
    # call the node externally with different parameters.
    parameters=[{'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str)}]
    )
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("delta_kinematics_controller"),
                    "urdf",
                    "delta_robot.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}
    # ros2_control_node (controller_manager) with mock hardware + spawner
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("delta_kinematics_controller"),
            "config",
            "demo_controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    spawner = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_my_delta',
        output='screen',
        arguments=['my_delta_controller', '--controller-manager', '/controller_manager'],
    )

    # Spawner for joint_state_broadcaster (publishes /joint_states)
    joint_state_broadcaster_spawner = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_joint_state_broadcaster',
        output='screen',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    # Spawner for joint_trajectory_controller (accepts /joint_trajectory_controller/follow_joint_trajectory actions)
    joint_trajectory_controller_spawner = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_joint_trajectory_controller',
        output='screen',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
    )

    ld.add_action(DeclareLaunchArgument('controller_yaml', default_value=default_yaml,
                                        description='Path to controller YAML configuration'))

    # Use provided YAML path by default (installed share area)
    # If installed, you can pass: --ros-args -p controller_yaml:=/path/to/demo_controllers.yaml
    ld.add_action(robot_state_pub)
    ld.add_action(control_node)
    ld.add_action(spawner)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(joint_trajectory_controller_spawner)

    return ld
