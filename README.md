# delta_kinematics_controller

`delta_kinematics_controller` is a ROS 2 `ros2_control` controller plugin that computes delta robot forward kinematics and publishes:

- End-effector pose (`ee_pose`)
- End-effector TF (optional)
- Additional joint states for lower-arm/EE visualization (`/joint_states`)

It uses a runtime kinematics plugin (`kinematics_interface_delta/DeltaKinematicsPlugin`) via `pluginlib`.

## Plugin

Controller type:

- `delta_kinematics_controller/DeltaKinematicsController`

Plugin definition:

- [resource/delta_kinematics_controller_plugins.xml](resource/delta_kinematics_controller_plugins.xml)

## Package layout

- Launch files: [launch](launch)
- Controller configs: [config](config)
- Source: [src](src)
- URDF/Xacro assets: [urdf](urdf), [description](description)
- Detailed docs: [doc](doc)

## Dependencies

This package depends on:

- `controller_interface`
- `hardware_interface`
- `pluginlib`
- `kinematics_interface`
- `kinematics_interface_delta`
- `rclcpp`, `rclcpp_lifecycle`
- `tf2_ros`, `geometry_msgs`, `sensor_msgs`

See [package.xml](package.xml) for the full list.

## Build

From workspace root:

```bash
colcon build --packages-select delta_kinematics_controller
source install/setup.bash
```

## Quick start

### Demo controller launch

```bash
ros2 launch delta_kinematics_controller demo_controller.launch.py
```

This launch brings up:

- `robot_state_publisher`
- `ros2_control_node`
- `my_delta_controller`
- `joint_state_broadcaster`
- `joint_trajectory_controller`
- `rviz2` (if available in your environment)

### Admittance demos

- [launch/admittance_demo.launch.py](launch/admittance_demo.launch.py)
- [launch/admittance_netft_demo.launch.py](launch/admittance_netft_demo.launch.py)

Related docs:

- [ADMITTANCE_QUICKSTART.md](ADMITTANCE_QUICKSTART.md)
- [doc/ADMITTANCE_DEMO.md](doc/ADMITTANCE_DEMO.md)
- [doc/NETFT_ADMITTANCE_DEMO.md](doc/NETFT_ADMITTANCE_DEMO.md)

## Core configuration

Default example config:

- [config/demo_controllers.yaml](config/demo_controllers.yaml)

Main `my_delta_controller` parameters:

- `joints`: 3 actuated delta joints (required)
- `lower_joints`: optional 6 passive joint names (`elbow_pitch/yaw_*`)
- `ee_joints`: optional prismatic EE chain joints (`ee_x`, `ee_y`, `ee_z`)
- `kinematics_plugin_name`: plugin class name
- `base_link`: TF parent frame
- `ee_link`: end-effector link name for FK
- `ee_tf_rate`: TF publish rate (Hz), `0` disables TF publishing
- `kinematics_interface_delta.*`: geometry parameters (`e`, `f`, `re`, `rf`, `motor_z_offset`)

## Topics

Published:

- `ee_pose` (`geometry_msgs/msg/PoseStamped`)
- `/joint_states` (`sensor_msgs/msg/JointState`) for passive/EE visualization values
- TF: `base_link -> ee_link` (when enabled and when `ee_joints` is not configured)

Consumed:

- Joint state interfaces from `ros2_control` for configured `joints`

## Notes

- If `ee_joints` is configured, EE translation is published via joint states so `robot_state_publisher` can produce EE TF from the kinematic chain.
- If `ee_joints` is empty, the controller publishes EE TF directly.
- For URDF integration guidance, see [URDF_SETUP.md](URDF_SETUP.md).

## Useful scripts

- [scripts/monitor_pose.py](scripts/monitor_pose.py)
- [scripts/test_trajectory.py](scripts/test_trajectory.py)
- [scripts/test_admittance_demo.py](scripts/test_admittance_demo.py)
- [scripts/publish_test_force.py](scripts/publish_test_force.py)

## License

Apache-2.0
