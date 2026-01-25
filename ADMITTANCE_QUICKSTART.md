# Admittance Controller Demo - Quick Start Guide

## What Was Created

I've created a complete admittance controller demo for your delta robot that implements a **chained controller architecture**:

### New Files Created:

1. **`config/admittance_demo_controllers.yaml`**
   - Complete controller configuration for the admittance demo
   - Configures: Joint State Broadcaster, Delta Kinematics Controller, Admittance Controller, Joint Trajectory Controller
   - All parameters documented with comments

2. **`launch/admittance_demo.launch.py`**
   - Launch file that starts all controllers in the correct order
   - Supports fake and real hardware modes
   - Properly chains controllers together

3. **`doc/ADMITTANCE_DEMO.md`**
   - Comprehensive documentation
   - Architecture explanation
   - Usage examples and troubleshooting

4. **`urdf/delta_fts_sensor.xacro`**
   - Example URDF for adding force-torque sensor
   - Shows both fake and real hardware configurations
   - Ready-to-use macros

5. **`scripts/test_admittance_demo.py`**
   - Automated test script
   - Verifies controller chain is working
   - Sends test commands and monitors responses

## Quick Start

### Step 1: Add Force-Torque Sensor to URDF

Edit your `delta_robot.urdf.xacro` and add inside the `<ros2_control>` section:

```xml
<sensor name="tcp_fts_sensor">
  <state_interface name="force.x"/>
  <state_interface name="force.y"/>
  <state_interface name="force.z"/>
  <state_interface name="torque.x"/>
  <state_interface name="torque.y"/>
  <state_interface name="torque.z"/>
</sensor>
```

### Step 2: Build the Package

```bash
cd ~/your_workspace
colcon build --packages-select delta_kinematics_controller
source install/setup.bash
```

### Step 3: Launch the Demo

```bash
ros2 launch delta_kinematics_controller admittance_demo.launch.py use_fake_hardware:=true
```

### Step 4: Test the Setup

In a new terminal:

```bash
source install/setup.bash
python3 src/delta_kinematics_controller/scripts/test_admittance_demo.py
```

## Controller Chain Architecture

```
User Command -> Admittance Controller -> Joint Trajectory Controller -> Hardware
                (exports position/vel)    (receives references)          (3 joints)
```

**Key Points:**
- Admittance controller provides force-compliant behavior
- Joint trajectory controller handles smooth motion execution
- Delta kinematics controller computes forward kinematics for visualization
- All controllers work together seamlessly

## Sending Commands

### Position Commands
```bash
ros2 topic pub /admittance_controller/joint_references trajectory_msgs/msg/JointTrajectoryPoint "{
  positions: [0.1, 0.0, -0.1]
}" --once
```

### Force Commands (Wrench)
```bash
ros2 topic pub /admittance_controller/wrench_reference geometry_msgs/msg/WrenchStamped "{
  header: {frame_id: 'end_effector'},
  wrench: {
    force: {x: 0.0, y: 0.0, z: 5.0}
  }
}" --rate 10
```

## Monitoring

```bash
# Check all controllers are active
ros2 control list_controllers

# Monitor admittance state
ros2 topic echo /admittance_controller/state

# Monitor joint states
ros2 topic echo /joint_states

# View in RViz
rviz2
```

## Key Configuration Parameters

Located in `config/admittance_demo_controllers.yaml`:

- **mass**: [5.0, 5.0, 5.0] - Virtual mass (lower = faster response)
- **stiffness**: [100.0, 100.0, 100.0] - Resistance to displacement
- **damping_ratio**: [1.0, 1.0, 1.0] - 1.0 = critically damped
- **filter_coefficient**: 0.05 - FT sensor filtering

## Tuning

### More Compliant (Softer)
```yaml
mass: [3.0, 3.0, 3.0, 1.0, 1.0, 1.0]
stiffness: [50.0, 50.0, 50.0, 10.0, 10.0, 10.0]
```

### More Stiff (Harder)
```yaml
mass: [10.0, 10.0, 10.0, 1.0, 1.0, 1.0]
stiffness: [200.0, 200.0, 200.0, 10.0, 10.0, 10.0]
```

### Reduce Oscillations
```yaml
damping_ratio: [2.0, 2.0, 2.0, 1.0, 1.0, 1.0]  # Overdamped
```

## Important Notes

1. **Original demo unchanged**: The original `demo_controllers.yaml` and `demo.launch.py` are not modified
2. **FT sensor required**: Must add force-torque sensor to URDF for admittance control
3. **Controller order matters**: Controllers must be spawned in correct sequence (see launch file)
4. **Test with fake hardware first**: Always test new configurations safely

## Troubleshooting

### "Could not find kinematics plugin"
Ensure kinematics_interface_delta is built:
```bash
colcon build --packages-select kinematics_interface_delta
```

### "No FT sensor data"
Verify FT sensor in URDF and check:
```bash
ros2 control list_hardware_interfaces | grep tcp_fts_sensor
```

### Controllers not loading
Check controller manager status:
```bash
ros2 control list_controllers
```

## Next Steps

1. Add force-torque sensor to your URDF
2. Test with fake hardware
3. Tune admittance parameters for your application
4. Deploy on real hardware with actual FT sensor
5. Implement force-controlled tasks

## Documentation

Full documentation: `doc/ADMITTANCE_DEMO.md`

Example URDF: `urdf/delta_fts_sensor.xacro`

## Support

For more information on admittance control:
- https://control.ros.org/jazzy/doc/ros2_controllers/admittance_controller/doc/userdoc.html

For controller chaining:
- https://control.ros.org/jazzy/doc/ros2_control/doc/index.html
