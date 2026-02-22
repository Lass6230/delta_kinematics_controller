# Delta Robot URDF Setup Guide

This document describes how the URDF must be structured for the
`DeltaKinematicsController` (and its companion kinematics plugin) to work
correctly.

> **Key concept:** A delta robot is a *closed-loop* parallel mechanism, but
> URDF only supports *open-loop* tree structures.  The URDF is therefore a
> **visualization aid** — the controller computes forward kinematics (FK)
> analytically and publishes the end-effector transform and passive joint
> angles at runtime.

---

## 1. Required Links

| Link | Purpose |
|------|---------|
| `base_link` | Fixed base platform. All arm joints are children of this link. |
| `upper_link_{N}` | Upper arm (driven by motor). One per arm. |
| `elbow_{N}` | Elbow marker (fixed to tip of upper arm). Visual only. |
| `lower_pitch_link_{N}` | Intermediate (zero-length) link between the two ball-joint DOFs. |
| `lower_link_{N}` | Lower arm (forearm). Visual extends along **−X** in the joint frame. |
| `lower_arm_end_{N}` | Marker at the tip of the lower arm (fixed to `lower_link`). |
| `ee` | End-effector platform. Connected to `base_link` via a floating joint. |

Where `{N}` is `1`, `2`, or `3` for each of the three arms.

### Optional links

| Link | Purpose |
|------|---------|
| `ee_attach_{N}` | Fixed children of `ee` representing the EE platform attachment points. Can be empty (no visual). |

---

## 2. Required Joints

### 2.1 Motor joints (actuated)

```
base_link  ──►  joint{N} (revolute, Y-axis)  ──►  upper_link_{N}
```

- **Type:** `revolute`
- **Axis:** `0 1 0` (local Y — tangent to the base circle)
- **Origin:** positioned on the base platform rim at the motor location, with
  `rpy="0 0 <yaw>"` to orient the arm radially outward.

The `yaw` values define each arm's radial direction.  The reference URDF uses:

| Arm | Yaw (rad) | Direction |
|-----|-----------|-----------|
| 1 | −π/2 ≈ −1.5708 | −Y |
| 2 | π/6 ≈ 0.5236 | 30° from +X |
| 3 | 5π/6 ≈ 2.6180 | 150° from +X |

### 2.2 Elbow (fixed)

```
upper_link_{N}  ──►  elbow_joint_{N} (fixed)  ──►  elbow_{N}
```

Placed at the tip of the upper arm: `<origin xyz="${rf} 0 0"/>`.

### 2.3 Passive ball-joint (two continuous joints)

The ball joint at each elbow is decomposed into **pitch then yaw**:

```
elbow_{N}  ──►  elbow_pitch_{N} (continuous, Y)  ──►  lower_pitch_link_{N}
                                                           │
                                              elbow_yaw_{N} (continuous, Z)
                                                           │
                                                      lower_link_{N}
```

| Joint | Axis | Purpose |
|-------|------|---------|
| `elbow_pitch_{N}` | `0 1 0` | Pitches the lower arm up/down in the arm's radial plane |
| `elbow_yaw_{N}` | `0 0 1` | Rotates the lower arm out of the radial plane |

These joints are **not actuated** — the controller computes their values from
forward kinematics and publishes them to `/joint_states`.
`robot_state_publisher` then positions the lower arm visuals accordingly.

### 2.4 Lower arm end (fixed)

```
lower_link_{N}  ──►  lower_arm_end_joint_{N} (fixed)  ──►  lower_arm_end_{N}
```

Placed at the tip of the lower arm: `<origin xyz="${-re} 0 0"/>` (lower arm
visual extends in **−X**).

### 2.5 End-effector (floating)

```
base_link  ──►  ee_floating (floating)  ──►  ee
```

- **Type:** `floating`
- `robot_state_publisher` does **not** publish TFs for floating joints.
- The controller broadcasts the `base_link → ee` transform on `/tf` using the
  FK-computed position.

---

## 3. Joint Naming Convention

The controller config maps joint names explicitly.  The names in the URDF
**must match** the names in the controller YAML:

```yaml
my_delta_controller:
  ros__parameters:
    joints: ["joint1", "joint2", "joint3"]
    lower_joints:
      - elbow_pitch_1
      - elbow_yaw_1
      - elbow_pitch_2
      - elbow_yaw_2
      - elbow_pitch_3
      - elbow_yaw_3
    base_link: "base_link"
    ee_link: "ee"
```

> **`ee_link` must match the URDF link name** for the end effector.  If the
> URDF link is `ee`, the config must say `ee_link: "ee"`.  A mismatch means
> the EE visual won't appear in RViz.

---

## 4. Lower Arm Visual Orientation

The lower arm visual (cylinder) must extend along **−X** in the
`lower_link_{N}` frame:

```xml
<origin xyz="${-re/2.0} 0 0" rpy="0 ${pi/2} 0"/>
```

The passive joint math in the controller assumes this convention:
- `elbow_pitch` rotates around Y
- `elbow_yaw` rotates around Z
- The arm direction is `[-1, 0, 0]` in the joint frame before rotation

---

## 5. Geometry Parameters

The controller and kinematics plugin share five geometry parameters.  These
must be consistent between the URDF dimensions and the config:

| Parameter | Description | Reference Value |
|-----------|-------------|-----------------|
| `e` | EE platform radius (half-side) | 0.045 m |
| `f` | Base platform radius (half-side) | 0.11812 m |
| `re` | Lower arm length (forearm) | 0.34 m |
| `rf` | Upper arm length | 0.17438 m |
| `motor_z_offset` | Vertical offset of motors from base origin | −0.025 m |

These appear in the config under:

```yaml
my_delta_controller:
  ros__parameters:
    kinematics_interface_delta:
      e: 0.045
      f: 0.11812
      re: 0.34
      rf: 0.17438
      motor_z_offset: -0.025
```

The URDF should use the **same values** for link/joint origins so the visuals
match the analytical FK.

---

## 6. ros2_control Hardware Interface

The URDF must include a `<ros2_control>` block declaring the three motor
joints with position command/state interfaces:

```xml
<ros2_control name="MockHardwareSystem" type="system">
  <hardware>
    <plugin>mock_components/GenericSystem</plugin>
  </hardware>
  <joint name="joint1">
    <command_interface name="position"/>
    <state_interface name="position">
      <param name="initial_value">0.0</param>
    </state_interface>
    <state_interface name="velocity"/>
  </joint>
  <!-- repeat for joint2, joint3 -->
</ros2_control>
```

Only the three **motor joints** are declared here.  The passive joints
(`elbow_pitch_*`, `elbow_yaw_*`) are **not** hardware interfaces — their
values are computed by the controller and published to `/joint_states`.

---

## 7. Complete Joint Chain Diagram

```
base_link
├── joint1 (revolute, Y)  →  upper_link_1
│   └── elbow_joint_1 (fixed)  →  elbow_1
│       └── elbow_pitch_1 (continuous, Y)  →  lower_pitch_link_1
│           └── elbow_yaw_1 (continuous, Z)  →  lower_link_1  [visual: cylinder along −X]
│               └── lower_arm_end_joint_1 (fixed)  →  lower_arm_end_1
├── joint2 (revolute, Y)  →  upper_link_2
│   └── ...same structure...
├── joint3 (revolute, Y)  →  upper_link_3
│   └── ...same structure...
└── ee_floating (floating)  →  ee  [visual: sphere, TF published by controller]
```

---

## 8. Common Pitfalls

| Problem | Cause | Fix |
|---------|-------|-----|
| EE sphere not visible in RViz | `ee_link` config doesn't match URDF link name | Set `ee_link` to match exactly (e.g. `"ee"`) |
| Lower arms don't move | `lower_joints` names don't match URDF joint names | Ensure pitch/yaw naming is identical |
| Lower arms point wrong direction | Visual origin not along −X | Use `xyz="${-re/2} 0 0" rpy="0 ${pi/2} 0"` |
| Arms visually detached from base | Geometry params in config differ from URDF origins | Keep `rf`, `f`, `motor_z_offset` consistent |
| Joint states not reaching robot_state_publisher | Controller publishes on namespaced topic | Controller must publish to absolute `"/joint_states"` |
