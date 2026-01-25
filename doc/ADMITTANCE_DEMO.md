# Admittance Controller Demo for Delta Robot

This demo demonstrates a chained controller setup using the Delta Kinematics Controller with an Admittance Controller and Joint Trajectory Controller.

## Overview

The admittance controller provides force-compliant behavior, allowing the robot to react to external forces measured by a force-torque sensor. This creates a safe, compliant robot that can:
- React to human interaction
- Perform contact-rich tasks
- Implement zero-force control
- Enable cooperative manipulation

## Controller Chain Architecture

```
┌──────────────┐     ┌────────────────────┐     ┌─────────────────────┐
│   Hardware   │────>│  Joint Trajectory  │────>│    Admittance       │
│  (3 joints)  │     │    Controller      │     │    Controller       │
└──────────────┘     └────────────────────┘     └─────────────────────┘
                              │                            │
                              │ reads position/velocity    │ exports position/velocity
                              │ writes position commands   │ references
                              │                            │
                              └────────────────────────────┘
                                   Chained Interface

User sends commands to: /admittance_controller/joint_references
Admittance controller outputs to: joint_trajectory_controller reference interfaces
Joint trajectory controller commands: hardware position interfaces
```

## Controllers Involved

1. **Joint State Broadcaster**: Publishes joint states to /joint_states topic
2. **Delta Kinematics Controller**: Computes forward kinematics and publishes lower joint states
3. **Admittance Controller**: Provides force-compliant control (exports position/velocity references)
4. **Joint Trajectory Controller**: Executes trajectories (receives references from admittance controller)

## Prerequisites

### Required Packages

```bash
sudo apt install ros-jazzy-admittance-controller ros-jazzy-kinematics-interface
```

### Force-Torque Sensor

The admittance controller requires a force-torque sensor in the URDF. You need to add:

```xml
<!-- Add to your delta_robot.urdf.xacro -->
<joint name="ft_sensor_joint" type="fixed">
  <parent link="end_effector"/>
  <child link="ft_sensor_frame"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>

<link name="ft_sensor_frame"/>

<!-- Add force-torque sensor to ros2_control -->
<sensor name="tcp_fts_sensor">
  <state_interface name="force.x"/>
  <state_interface name="force.y"/>
  <state_interface name="force.z"/>
  <state_interface name="torque.x"/>
  <state_interface name="torque.y"/>
  <state_interface name="torque.z"/>
</sensor>
```

## Running the Demo

### With Fake Hardware

```bash
cd ~/your_workspace
source install/setup.bash
ros2 launch delta_kinematics_controller admittance_demo.launch.py use_fake_hardware:=true
```

### With Real Hardware

```bash
ros2 launch delta_kinematics_controller admittance_demo.launch.py use_fake_hardware:=false
```

## Sending Commands

### Using Joint References Topic

Send position commands to the admittance controller:

```bash
ros2 topic pub /admittance_controller/joint_references trajectory_msgs/msg/JointTrajectoryPoint "{
  positions: [0.0, 0.0, 0.0],
  velocities: [0.0, 0.0, 0.0],
  accelerations: [0.0, 0.0, 0.0]
}" --once
```

### Using Wrench Reference (Force Commands)

Apply an offset wrench in the FT sensor frame:

```bash
ros2 topic pub /admittance_controller/wrench_reference geometry_msgs/msg/WrenchStamped "{
  header: {
    frame_id: 'end_effector'
  },
  wrench: {
    force: {x: 0.0, y: 0.0, z: -5.0},
    torque: {x: 0.0, y: 0.0, z: 0.0}
  }
}" --rate 10
```

This will make the robot compliant to a -5N force in the Z direction.

## Monitoring

### Check Controller Status

```bash
ros2 control list_controllers
```

Expected output:
```
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
my_delta_controller[delta_kinematics_controller/DeltaKinematicsController] active
joint_trajectory_controller[joint_trajectory_controller/JointTrajectoryController] active
admittance_controller[admittance_controller/AdmittanceController] active
```

### Monitor Admittance Controller State

```bash
ros2 topic echo /admittance_controller/state
```

### Monitor Force-Torque Sensor

```bash
ros2 topic echo /tcp_fts_sensor/wrench
```

## Configuration Parameters

### Key Admittance Parameters (in `admittance_demo_controllers.yaml`)

- **mass**: Virtual mass for each axis (lower = more responsive)
- **damping_ratio**: Damping behavior (1.0 = critically damped)
- **stiffness**: Resistance to displacement (higher = stiffer)
- **selected_axes**: Enable/disable admittance for each axis
- **filter_coefficient**: FT sensor filtering (lower = more filtering)

### Tuning Tips

1. **Increase Compliance**: Lower mass, lower stiffness
2. **Reduce Oscillations**: Increase damping_ratio
3. **Faster Response**: Lower mass, lower filter_coefficient
4. **Smoother Behavior**: Higher mass, higher damping_ratio

## Troubleshooting

### Admittance Controller Fails to Load

**Symptom**: Controller fails with "Could not find kinematics plugin"

**Solution**: Ensure kinematics_interface_delta is installed and built:
```bash
colcon build --packages-select kinematics_interface_delta
source install/setup.bash
```

### No Force-Torque Data

**Symptom**: Admittance controller loaded but no compliance behavior

**Solution**: 
1. Check FT sensor is publishing: `ros2 topic echo /tcp_fts_sensor/wrench`
2. Verify FT sensor name in URDF matches config (`tcp_fts_sensor`)
3. Check FT sensor frame exists in URDF

### Controller Chain Not Working

**Symptom**: Controllers load but commands don't reach hardware

**Solution**: Check controller chain order:
```bash
ros2 control list_hardware_interfaces
```

Verify reference interfaces are exported:
```
admittance_controller/joint1/position [available] [claimed]
admittance_controller/joint2/position [available] [claimed]
...
```

## Advanced Usage

### Dynamic Parameter Updates

You can update admittance parameters at runtime:

```bash
ros2 param set /admittance_controller admittance.mass "[3.0, 3.0, 3.0, 1.0, 1.0, 1.0]"
ros2 param set /admittance_controller admittance.stiffness "[50.0, 50.0, 50.0, 10.0, 10.0, 10.0]"
```

### Different Control Modes

1. **Pure Force Control**: Set high masses and low stiffness
2. **Position Control with Compliance**: Medium masses and stiffness
3. **Stiff Position Control**: Low masses and high stiffness

## Safety Considerations

1. **Test with Fake Hardware First**: Always test new parameters with `use_fake_hardware:=true`
2. **Start with High Masses**: Begin with high virtual masses for slower, safer response
3. **Monitor Forces**: Always monitor FT sensor data to ensure safe operation
4. **E-Stop Ready**: Have emergency stop accessible when testing with real hardware
5. **Workspace Limits**: Ensure admittance parameters don't allow motion outside safe workspace

## References

- [ROS 2 Admittance Controller Documentation](https://control.ros.org/jazzy/doc/ros2_controllers/admittance_controller/doc/userdoc.html)
- [Controller Chaining in ros2_control](https://control.ros.org/jazzy/doc/ros2_control/doc/index.html)
- [Kinematics Interface](https://github.com/ros-controls/kinematics_interface)
