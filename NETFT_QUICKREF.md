# Network FT Sensor + Admittance Control - Quick Reference

## 🚀 Quick Start

### With Fake Hardware (Testing)
```bash
ros2 launch delta_kinematics_controller admittance_netft_demo.launch.py
```

### With Real ATI FT Sensor
```bash
ros2 launch delta_kinematics_controller admittance_netft_demo.launch.py \
  use_fake_hardware:=false \
  ft_sensor_ip:=192.168.1.1 \
  ft_sensor_type:=ati
```

### With OnRobot HEX-E Sensor
```bash
ros2 launch delta_kinematics_controller admittance_netft_demo.launch.py \
  use_fake_hardware:=false \
  ft_sensor_ip:=192.168.1.10 \
  ft_sensor_type:=onrobot
```

## 📊 Monitoring

```bash
# Check all controllers
ros2 control list_controllers

# View FT sensor data
ros2 topic echo /ft_data

# View admittance state
ros2 topic echo /admittance_controller/state

# Check sensor diagnostics
ros2 topic echo /diagnostics | grep tcp_fts_sensor
```

## 🎮 Commands

### Zero the Sensor
```bash
ros2 service call /tcp_fts_sensor/zero std_srvs/srv/Trigger
```

### Send Position Command
```bash
ros2 topic pub /admittance_controller/joint_references \
  trajectory_msgs/msg/JointTrajectoryPoint "{positions: [0.1, 0.0, 0.0]}" --once
```

### Apply Force Offset
```bash
ros2 topic pub /admittance_controller/wrench_reference \
  geometry_msgs/msg/WrenchStamped \
  "{header: {frame_id: 'end_effector'}, wrench: {force: {z: 5.0}}}" --rate 10
```

## 🔧 Common Parameters

| Parameter | Description | Default |
|-----------|-------------|---------|
| `ft_sensor_ip` | Sensor IP address | `192.168.1.1` |
| `ft_sensor_type` | `ati`, `ati_axia`, or `onrobot` | `ati` |
| `ft_rdt_sampling_rate` | Sampling rate (Hz) | `1000` |
| `ft_filter_rate` | Filter rate (Hz) | `100` |

## 🐛 Troubleshooting

### Sensor Not Connecting
```bash
ping 192.168.1.1
ros2 launch net_ft_driver net_ft_broadcaster.launch.py
```

### No Force Data
```bash
ros2 topic hz /ft_data
ros2 control list_hardware_interfaces | grep tcp_fts_sensor
```

### Noisy Readings
```bash
# Zero the sensor
ros2 service call /tcp_fts_sensor/zero std_srvs/srv/Trigger

# Or increase filtering in config:
# ft_sensor.filter_coefficient: 0.1
```

## 📁 New Files Created

1. `config/admittance_netft_controllers.yaml` - Controller config with FT sensor
2. `launch/admittance_netft_demo.launch.py` - Launch with net_ft_driver
3. `urdf/delta_with_netft.urdf.xacro` - URDF with network FT sensor
4. `doc/NETFT_ADMITTANCE_DEMO.md` - Full documentation

## 🎯 Controller Chain

```
Network FT Sensor (UDP)
  ↓
net_ft_driver Hardware Interface
  ↓
Force Torque Sensor Broadcaster → /ft_data
  ↓
Admittance Controller (reads tcp_fts_sensor)
  ↓
Joint Trajectory Controller
  ↓
Delta Robot Hardware
```

## ✅ Verification Steps

1. **Test sensor standalone:**
   ```bash
   ros2 launch net_ft_driver net_ft_broadcaster.launch.py
   ros2 topic echo /ft_data
   ```

2. **Launch full demo:**
   ```bash
   ros2 launch delta_kinematics_controller admittance_netft_demo.launch.py use_fake_hardware:=false
   ```

3. **Zero sensor:**
   ```bash
   ros2 service call /tcp_fts_sensor/zero std_srvs/srv/Trigger
   ```

4. **Check controllers:**
   ```bash
   ros2 control list_controllers
   ```

5. **Send test command:**
   ```bash
   ros2 topic pub /admittance_controller/joint_references \
     trajectory_msgs/msg/JointTrajectoryPoint "{positions: [0.05, 0.0, 0.0]}" --once
   ```

6. **Test compliance:** Gently push end effector, robot should yield

## 📚 Documentation

- **Full Guide:** `doc/NETFT_ADMITTANCE_DEMO.md`
- **Original Admittance Demo:** `doc/ADMITTANCE_DEMO.md`
- **Quick Start:** `ADMITTANCE_QUICKSTART.md`
