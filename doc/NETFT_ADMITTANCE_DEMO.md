# Delta Robot with Network FT Sensor - Admittance Control Demo

This demo integrates your **Network-based Force-Torque Sensor** (from `ros2_net_ft_driver`) with the delta robot for admittance control.

## Overview

This setup uses:
- **Delta Robot**: 3-DOF parallel manipulator
- **Network FT Sensor**: ATI, ATI Axia, or OnRobot force-torque sensor
- **Admittance Controller**: Force-compliant control based on measured forces
- **Joint Trajectory Controller**: Smooth trajectory execution

## Supported FT Sensors

The `net_ft_driver` supports:
- **ATI Force-Torque Sensors** (Net F/T series)
- **ATI Axia Sensors**
- **OnRobot HEX-E Force-Torque Sensors**

## Architecture

```
┌─────────────────┐
│  Network FT     │ (Ethernet connection)
│  Sensor         │
│  (ATI/OnRobot)  │
└────────┬────────┘
         │ UDP packets
         ▼
┌─────────────────────────────┐
│  net_ft_driver              │
│  Hardware Interface         │
└────────┬────────────────────┘
         │ ros2_control interfaces
         ▼
┌─────────────────────────────┐
│  Force Torque Sensor        │
│  Broadcaster                │ ──> /ft_data (WrenchStamped)
└────────┬────────────────────┘
         │
         ▼
┌─────────────────────────────┐
│  Admittance Controller      │ <── reads tcp_fts_sensor interfaces
│  (force-compliant control)  │
└────────┬────────────────────┘
         │ position/velocity references
         ▼
┌─────────────────────────────┐
│  Joint Trajectory           │
│  Controller                 │
└────────┬────────────────────┘
         │ position commands
         ▼
┌─────────────────────────────┐
│  Delta Robot Hardware       │
└─────────────────────────────┘
```

## Hardware Setup

### 1. Network Configuration

Connect your FT sensor to your network and configure:

**ATI Sensor Default:**
- IP Address: `192.168.1.1`
- Port: `49152` (RDT protocol)

**OnRobot Sensor:**
- Check sensor documentation for default IP

### 2. Test Network Connection

```bash
# Ping the sensor
ping 192.168.1.1

# Check if sensor is responding on RDT port
nc -u -v 192.168.1.1 49152
```

### 3. Verify Sensor Communication

```bash
# Build the net_ft_driver package
cd ~/your_workspace
colcon build --packages-select net_ft_driver net_ft_description net_ft_diagnostic_broadcaster

# Test with standalone net_ft broadcaster
ros2 launch net_ft_driver net_ft_broadcaster.launch.py
```

## Configuration

### Launch Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `use_fake_hardware` | `true` | Use fake hardware for testing |
| `ft_sensor_ip` | `192.168.1.1` | IP address of FT sensor |
| `ft_sensor_type` | `ati` | Sensor type: `ati`, `ati_axia`, or `onrobot` |
| `ft_rdt_sampling_rate` | `1000` | Sampling rate in Hz |
| `ft_filter_rate` | `100` | Internal filter rate in Hz |
| `ft_use_hardware_biasing` | `true` | Use hardware biasing/zeroing |

### Sensor Types

**ATI Standard:**
```bash
ft_sensor_type:=ati
```

**ATI Axia:**
```bash
ft_sensor_type:=ati_axia
```

**OnRobot HEX-E:**
```bash
ft_sensor_type:=onrobot
```

## Running the Demo

### With Fake Hardware (Testing)

```bash
cd ~/your_workspace
source install/setup.bash

ros2 launch delta_kinematics_controller admittance_netft_demo.launch.py \
  use_fake_hardware:=true
```

### With Real Network FT Sensor

```bash
ros2 launch delta_kinematics_controller admittance_netft_demo.launch.py \
  use_fake_hardware:=false \
  ft_sensor_ip:=192.168.1.1 \
  ft_sensor_type:=ati \
  ft_rdt_sampling_rate:=1000 \
  ft_filter_rate:=100
```

### With OnRobot Sensor

```bash
ros2 launch delta_kinematics_controller admittance_netft_demo.launch.py \
  use_fake_hardware:=false \
  ft_sensor_ip:=192.168.1.10 \
  ft_sensor_type:=onrobot \
  ft_rdt_sampling_rate:=500
```

## Monitoring

### Check Controllers

```bash
ros2 control list_controllers
```

Expected output:
```
joint_state_broadcaster                  [active]
force_torque_sensor_broadcaster          [active]
net_ft_diagnostic_broadcaster            [active]
my_delta_controller                      [active]
joint_trajectory_controller              [active]
admittance_controller                    [active]
```

### Monitor FT Sensor Data

```bash
# View force-torque measurements
ros2 topic echo /ft_data

# View sensor diagnostics
ros2 topic echo /diagnostics

# View raw sensor state interfaces
ros2 control list_hardware_interfaces | grep tcp_fts_sensor
```

Expected interfaces:
```
tcp_fts_sensor/force.x [available] [claimed]
tcp_fts_sensor/force.y [available] [claimed]
tcp_fts_sensor/force.z [available] [claimed]
tcp_fts_sensor/torque.x [available] [claimed]
tcp_fts_sensor/torque.y [available] [claimed]
tcp_fts_sensor/torque.z [available] [claimed]
```

### Monitor Admittance Controller

```bash
# View admittance controller state
ros2 topic echo /admittance_controller/state

# View measured wrench being used for admittance
ros2 topic echo /admittance_controller/state | grep wrench
```

## Sensor Calibration

### Hardware Biasing (Zeroing)

The sensor can be zeroed to remove static offsets:

```bash
# Zero the sensor (requires use_hardware_biasing:=true)
ros2 service call /tcp_fts_sensor/zero std_srvs/srv/Trigger
```

### Software Biasing

If hardware biasing is disabled, you can use software filtering in the admittance controller:

```yaml
admittance_controller:
  ros__parameters:
    ft_sensor:
      filter_coefficient: 0.1  # Increase for more filtering
```

## Sending Commands

### Position Commands

```bash
ros2 topic pub /admittance_controller/joint_references trajectory_msgs/msg/JointTrajectoryPoint "{
  positions: [0.1, 0.0, 0.0]
}" --once
```

### Wrench Commands (Force Offset)

```bash
ros2 topic pub /admittance_controller/wrench_reference geometry_msgs/msg/WrenchStamped "{
  header: {frame_id: 'end_effector'},
  wrench: {
    force: {x: 0.0, y: 0.0, z: 5.0}
  }
}" --rate 10
```

## Troubleshooting

### FT Sensor Not Connecting

**Symptom:** `net_ft_driver` fails to start or no data on `/ft_data`

**Solutions:**
1. Check network connection: `ping <sensor_ip>`
2. Verify IP address matches sensor configuration
3. Check firewall settings (UDP port 49152)
4. Ensure sensor is powered on
5. Try different network interface if multiple available

```bash
# Check which interface to use
ip addr show

# Specify network interface in launch (if needed)
# Add to hardware interface plugin parameters
```

### No Force Data in Admittance Controller

**Symptom:** Admittance controller loaded but no compliance

**Solutions:**
1. Verify FT sensor broadcaster is active:
   ```bash
   ros2 control list_controllers | grep force_torque
   ```

2. Check sensor data is publishing:
   ```bash
   ros2 topic hz /ft_data
   ```

3. Verify sensor name matches in both URDF and admittance config (`tcp_fts_sensor`)

4. Check hardware interfaces are claimed:
   ```bash
   ros2 control list_hardware_interfaces | grep tcp_fts_sensor
   ```

### Sensor Reading Noise or Drift

**Symptom:** Noisy force readings or sensor drift over time

**Solutions:**
1. **Zero the sensor** before use:
   ```bash
   ros2 service call /tcp_fts_sensor/zero std_srvs/srv/Trigger
   ```

2. **Increase filter coefficient** in admittance controller config:
   ```yaml
   ft_sensor:
     filter_coefficient: 0.1  # More filtering (was 0.05)
   ```

3. **Lower internal filter rate** when launching:
   ```bash
   ft_filter_rate:=50  # Lower rate = more filtering
   ```

4. **Check sensor mounting** - ensure sensor is rigidly mounted

5. **Verify cable connections** - loose connections cause noise

### High Latency or Packet Loss

**Symptom:** Diagnostic shows high packet loss or out-of-order packets

**Solutions:**
1. Check network quality:
   ```bash
   ros2 topic echo /diagnostics | grep lost_packets
   ```

2. **Reduce sampling rate**:
   ```bash
   ft_rdt_sampling_rate:=500  # Lower rate
   ```

3. **Use wired connection** instead of WiFi

4. **Reduce network traffic** on same interface

5. **Check CPU load** - high system load causes packet drops

## Sensor Diagnostics

The `net_ft_diagnostic_broadcaster` publishes diagnostic information:

```bash
ros2 topic echo /diagnostics
```

Key diagnostic fields:
- **packet_count**: Total packets received
- **lost_packets**: Number of lost packets
- **out_of_order_count**: Packets received out of sequence
- **status**: Overall sensor status

## Advanced Configuration

### High-Speed Sampling

For high-bandwidth force control:

```bash
ros2 launch delta_kinematics_controller admittance_netft_demo.launch.py \
  use_fake_hardware:=false \
  ft_sensor_ip:=192.168.1.1 \
  ft_rdt_sampling_rate:=7000 \
  ft_filter_rate:=1000
```

**Note:** Higher sampling rates require:
- Low-latency network connection
- Sufficient CPU resources
- May increase packet loss if network is congested

### Custom Admittance Parameters

Edit `config/admittance_netft_controllers.yaml`:

```yaml
admittance_controller:
  ros__parameters:
    admittance:
      mass: [2.0, 2.0, 2.0, 1.0, 1.0, 1.0]      # Lower mass = more responsive
      stiffness: [50.0, 50.0, 50.0, 10.0, 10.0, 10.0]  # Lower stiffness = more compliant
      damping_ratio: [0.7, 0.7, 0.7, 1.0, 1.0, 1.0]    # Underdamped = faster but may oscillate
```

## Safety Considerations

1. **Always zero the sensor** before operation to remove static loads
2. **Start with high masses and stiffness** for safer, slower response
3. **Test with fake hardware first** before connecting real FT sensor
4. **Monitor diagnostics** for sensor health and data quality
5. **Set workspace limits** to prevent unsafe motions
6. **Have emergency stop** accessible during testing
7. **Check sensor mounting** is secure and properly aligned
8. **Verify force measurements** match expected values before admittance control

## Performance Tips

1. **Network Quality**: Use dedicated network interface for FT sensor
2. **Real-Time**: Consider real-time kernel for best performance
3. **CPU Affinity**: Pin control_node to specific CPU cores
4. **Filtering**: Balance between responsiveness and noise reduction
5. **Sampling Rate**: Match to control loop frequency (typically 100-1000 Hz)

## Example: Complete Setup Procedure

```bash
# 1. Build workspace
cd ~/a6_servo_ws
colcon build --packages-select net_ft_driver net_ft_description \
  net_ft_diagnostic_broadcaster delta_kinematics_controller

# 2. Source workspace
source install/setup.bash

# 3. Test FT sensor standalone
ros2 launch net_ft_driver net_ft_broadcaster.launch.py

# In another terminal:
ros2 topic echo /ft_data

# If sensor working, proceed:

# 4. Launch admittance demo
ros2 launch delta_kinematics_controller admittance_netft_demo.launch.py \
  use_fake_hardware:=false \
  ft_sensor_ip:=192.168.1.1 \
  ft_sensor_type:=ati

# 5. Zero the sensor
ros2 service call /tcp_fts_sensor/zero std_srvs/srv/Trigger

# 6. Verify controllers
ros2 control list_controllers

# 7. Test with position command
ros2 topic pub /admittance_controller/joint_references \
  trajectory_msgs/msg/JointTrajectoryPoint "{positions: [0.05, 0.0, 0.0]}" --once

# 8. Monitor compliance (gently push end effector and observe motion)
ros2 topic echo /ft_data
```

## References

- [ros2_net_ft_driver GitHub](https://github.com/ICube-Robotics/ros2_net_ft_driver)
- [ATI Industrial Automation - Net F/T](https://www.ati-ia.com/products/ft/ft_models.aspx)
- [OnRobot HEX-E](https://onrobot.com/en/products/hex-e-force-torque-sensor)
- [ROS 2 Admittance Controller](https://control.ros.org/jazzy/doc/ros2_controllers/admittance_controller/doc/userdoc.html)
