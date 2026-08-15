# GimbalStabilizer Node

## Purpose
Stabilizes a 3-axis gimbal camera by counteracting vehicle rotation while tracking commanded gimbal angles. Computes roll/pitch compensation from vehicle IMU data and outputs joint position commands to Isaac Sim.

## Subscriptions
- `isaac_joint_states` (`sensor_msgs/msg/JointState`) — current gimbal joint positions from Isaac Sim
- `state/pose` (`geometry_msgs/msg/PoseStamped`) — vehicle pose
- `mavros/imu/data` (`sensor_msgs/msg/Imu`) — vehicle IMU data (orientation for stabilization)
- `gimbal_command_rpy_deg` (`geometry_msgs/msg/Vector3`) — desired gimbal angles in degrees

## Publishers
- `isaac_joint_commands` (`sensor_msgs/msg/JointState`) — stabilized joint position commands
- `gimbal_state_rpy_rad` (`geometry_msgs/msg/Vector3`) — current gimbal RPY in radians
- `gimbal_state_rpy_deg` (`geometry_msgs/msg/Vector3`) — current gimbal RPY in degrees

## Parameters
- `gimbal_angle_order` (`string`, default: `zyx`) — Euler angle convention
- `mounting_orientation` (`dict`) — roll/pitch/yaw mounting offsets
- `joint_limits` (`dict`) — min/max angles and max rates per joint
- `filter` (`dict`) — low-pass filter cutoff frequency (default 0.1 Hz)

## Dependencies
Subscribes to `mavros/imu/data` published by offboard_py. Subscribes to `isaac_joint_states` from Isaac Sim. Publishes `isaac_joint_commands` consumed by Isaac Sim.

## Key Files
- `gimbal_stabilizer/gimbal_stabilizer.py` — Node implementation
- `config/gimbal_config.yaml` — Gimbal parameters (mounting, limits, filter)
- `config/vehicles.yaml` — Multi-agent vehicle configuration
- `launch/multi_agent_gimbal.launch.py` — Multi-agent launch file

## Calling Contract

**Pattern**: Decoupled (subscribe → cache, timer → publish)

- `joint_state_callback()`: Caches current joint positions. No publishing.
- `pose_callback()`: Caches vehicle pose. No publishing.
- `imu_callback()`: Caches vehicle orientation. No publishing.
- `gimbal_command_callback()`: Caches desired gimbal angles. No publishing.
- `timer_callback()` (100 Hz): Computes stabilized angles, publishes joint commands and gimbal state. Sole periodic mutation point.

## Joint Mapping
- `cgo3_vertical_arm_joint` — controls yaw (has +pi/2 offset)
- `cgo3_horizontal_arm_joint` — controls roll
- `cgo3_camera_joint` — controls pitch

---

# LOSRateController Node

## Purpose
World-frame LOS (Line-of-Sight) gimbal controller for RL policy deployment. Ports the iris_ma6 Jacobian-inverse gimbal controller to ROS2. Two modes:

- **Rate mode** (default): Accepts normalized azimuth/elevation rate commands (matching RL policy action space), integrates into persistent world-frame target, computes body-frame joint positions via unified J^{-1} control law.
- **Position mode**: Accepts world-frame azimuth/elevation position targets (from gimbal_los_tracker), computes body-frame joints via analytical atan2 IK.

## Subscriptions
- `gimbal_cmd_los_rate` (`geometry_msgs/msg/Vector3`) — LOS rate commands in rad/s: x=azimuth_rate, y=elevation_rate (rate mode)
- `gimbal_cmd_los_world_deg` (`geometry_msgs/msg/Vector3`) — world-frame position target: z=azimuth, y=elevation [deg] (position mode)
- `mavros/imu/data` (`sensor_msgs/msg/Imu`) — vehicle IMU (quaternion + angular velocity for stabilization)
- `isaac_joint_states` (`sensor_msgs/msg/JointState`) — actual joint positions (feedback)
- `zoom_rate_cmd` (`std_msgs/msg/Float32`) — zoom rate command in zoom-levels/s

## Publishers
- `isaac_joint_commands` (`sensor_msgs/msg/JointState`) — joint targets (position and/or velocity)
- `gimbal_state_rpy_deg` (`geometry_msgs/msg/Vector3`) — actual body-frame RPY in degrees (from joint feedback)
- `gimbal_los_state_deg` (`geometry_msgs/msg/Vector3`) — world-frame azimuth (x) / elevation (y) in degrees
- `combined_ang_vel_w` (`geometry_msgs/msg/Vector3Stamped`) — body + gimbal angular velocity in world frame
- `camera/zoom_level` (`std_msgs/msg/Float64`) — current zoom level state
- `camera/zoom_level_cmd` (`std_msgs/msg/Float64`) — zoom level command to sim camera

## Parameters
- `model` (`string`, default: `iris_gimbal3`) — gimbal model, selects joint name mapping
- `control_mode` (`string`, default: `position`) — actuator command: `position`, `position_velocity`, or `velocity`
- `control_trigger` (`string`, default: `joint_states`) — what triggers control loop: `joint_states` (sim-time dedup) or `imu` (higher rate)
- `pointing_gain` (`float`, default: `32.5`) — proportional gain K for J^{-1} attitude error (matches iris_ma6 training)
- `servo_rate_limit` (`float`, default: `0.0`) — max omega_cmd per-component [rad/s] before J^{-1}. 0 = use max_gimbal_rate. Prevents overshoot with delayed sim feedback.
- `max_gimbal_rate` (`float`, default: `π`) — max gimbal angular rate [rad/s], used for LOS rate normalization
- `yaw_limits_deg` (`list[float]`, default: `[-270, 270]`)
- `pitch_limits_deg` (`list[float]`, default: `[-45, 45]`)
- `roll_limits_deg` (`list[float]`, default: `[-45, 45]`)
- `update_rate` (`float`, default: `100.0`) — nominal control rate [Hz]

## Dependencies
Subscribes to `mavros/imu/data` published by offboard_py. Subscribes to `isaac_joint_states` from PegasusSimulator OmniGraph. Publishes `isaac_joint_commands` consumed by PegasusSimulator OmniGraph (`ROS2SubscribeJointState` → `IsaacArticulationController`). **Mutually exclusive** with gimbal_stabilizer — only one should run per vehicle.

## Key Files
- `gimbal_stabilizer/los_rate_controller.py` — Node implementation
- `config/los_rate_config.yaml` — Default parameters
- `launch/multi_agent_los_rate.launch.py` — Multi-agent launch file

## Calling Contract

**Pattern**: Event-driven (no timer — control runs in subscriber callbacks)

- `_los_rate_cmd_callback()`: Caches normalized rate commands. Clears position target (switches to rate mode).
- `_los_world_cmd_callback()`: Caches world-frame position target. Clears rate commands (switches to position mode).
- `_imu_callback()`: Caches vehicle quaternion (xyzw→wxyz) + angular velocity. Triggers control if `control_trigger=imu`.
- `_joint_state_callback()`: Caches actual joint positions. Deduplicates by sim timestamp. Triggers control if `control_trigger=joint_states`.
- `_run_control(dt)`: Core control law. Position mode: atan2 IK. Rate mode: J^{-1}(−K·att_error − ω_body). Publishes joint commands + state.

## Control Law (Rate Mode)
```
1. Integrate LOS rate → persistent world-frame az/el target
2. Desired camera quaternion in body frame (Gram-Schmidt)
3. Current gimbal quaternion from actual joints
4. Quaternion error → body-frame att_error
5. omega_cmd = clip(-K * att_error, ±servo_rate_limit)
6. qdot = J^{-1} * (omega_cmd - omega_body)
7. q_ref = actual + qdot * dt → clamp to limits
```

## Joint Mapping
Resolved at startup from `model` parameter (YAW_JOINT_OFFSET = +π/2 applied at publish):

| Model | Yaw | Roll | Pitch |
|-------|-----|------|-------|
| `cgo3` | `cgo3_vertical_arm_joint` | `cgo3_horizontal_arm_joint` | `cgo3_camera_joint` |
| `iris_gimbal3` | `yaw_joint` | `roll_joint` | `pitch_joint` |

## Reference Implementation
Ported from `IsaacLab/.../iris_ma6/controller/gimbal_controller_jacobian.py`

---

# JointStatePublisher Node

## Purpose
Simple test/example node that publishes fixed joint angles at 100 Hz. Used for development and debugging.

## Publishers
- `/px4_1/isaac_joint_commands` (`sensor_msgs/msg/JointState`) — fixed joint position commands

## Dependencies
None (standalone node).

## Key Files
- `gimbal_stabilizer/joint_state_publisher.py` — Node implementation

## Calling Contract

**Pattern**: Timer-only (no subscriptions)

- `timer_callback()` (100 Hz): Publishes fixed joint positions. Stateless.

---

# FixedMountPublisher Node

## Purpose
Sim stand-in for the real interceptor's **rigidly bolted, non-gimballed** camera (MAS ticket
055). The aircraft has no gimbal and no zoom hardware: the camera is bolted forward and pitched
30.8° UP in body-FLU, and the field stack simply shims two constants onto the wire
(`mas_fieldtest/src/tmux/intercept_mas.yaml:119-120`). This node is those two shims plus the
constant `isaac_joint_commands` the sim additionally needs to hold Isaac's gimbal at the
equivalent pose.

It **replaces `los_rate_controller`** in the fixed-camera session. As with the other two
controllers, only one may run per vehicle — they all publish `isaac_joint_commands`.

Constants, not control, is the point: a rigid mount has no tracking dynamics, no rate loop and
no body-motion rejection, so anything computed per tick would be a sim artifact with no
counterpart in the aircraft.

## Subscriptions
- `isaac_joint_states` (`sensor_msgs/msg/JointState`) — feedback, used **only** by the
  deflection monitor (`monitor_deflection: true`). Nothing on the output path reads it.

## Publishers
- `isaac_joint_commands` (`sensor_msgs/msg/JointState`) — constant joint positions (RELIABLE, depth 10)
- `gimbal_state_rpy_deg` (`geometry_msgs/msg/Vector3`) — the constant mount pose in degrees
- `camera/zoom_level` (`std_msgs/msg/Float64`) — constant, default 1.0 (suppressible)
- `gimbal_mount_deflection_deg` (`geometry_msgs/msg/Vector3`) — peak-hold |achieved − commanded|
  per joint, `(x=roll, y=pitch, z=yaw)`; ticket 055 AC4 instrumentation

**Deliberately NOT published: `camera/zoom_level_cmd`.** Pegasus's `ROS2Backend` subscribes it
and calls `MonocularCamera.set_zoom`, which rebuilds the camera's focal length as
`mean(fx, fy)` — a no-op on a square-pixel camera, and a silent 444.76 → 388.87 px
miscalibration on the interceptor's anisotropic one (ticket 055 F2). `los_rate_controller`
publishes it every tick; this node must not.

## Parameters
- `mount_pitch_up_deg` (`float`, default: `30.8`) — mount pitch, **positive UP** (the field
  convention), negated once internally
- `mount_roll_deg` (`float`, default: `0.0`)
- `mount_yaw_deg` (`float`, default: `0.0`) — +left about body +Z; 0 = forward
- `model` (`string`, default: `iris_gimbal3`) — selects the joint-name mapping
- `publish_rate_hz` (`float`, default: `50.0`)
- `zoom_level` (`float`, default: `1.0`) / `publish_zoom_level` (`bool`, default: `true`)
- `monitor_deflection` (`bool`, default: `true`) / `deflection_warn_deg` (`float`, default: `0.5`)
- `check_boot_label` (`bool`, default: `true`) / `boot_label_path` (`string`, default:
  `/tmp/isaac_boot_label.json`) — the Isaac/ROS consistency guard, below

## Two consistency guards (ticket 055 D2)

A mixed session picks the fixed-camera vehicles **twice**: on the Isaac side via
`FIXED_CAMERA_VEHICLES`, on the ROS side via this node's launch `namespaces:=`. Two lists, two
delivery mechanisms. A disagreement is silent and looks like a *tracking* bug rather than a
*config* bug — fisheye intrinsics on a gimbal being swung by a LOS-rate controller, or a rigid
mount still wearing the 1053 px pinhole. And it is likely: `VAR=... tmuxp load` is silently
dropped whenever a tmux server exists, and an idle `keepalive` session keeps one alive on this
box permanently.

1. **Boot label** (`_check_boot_label`, at startup). `px4_multi_world_iris_fixedcam.isaac.py`
   writes its selection into `/tmp/isaac_boot_label.json` as `fixed_camera_vehicles`; this node
   reads it back and **raises** if its own vehicle id is absent. The label carries the writing
   process's `pid`, and a label whose pid is not alive is treated as no label at all — a stale
   file from a previous session must never authorise anything. Fail-closed on *disagreement*,
   warn-only when the label is missing, stale, or predates ticket 055, so the node stays usable
   against a hand-started sim. `check_boot_label:=false` skips it.
2. **Command contention** (`_check_command_contention`, ~5 s after startup). Counts publishers
   on `isaac_joint_commands`; more than one means a `los_rate_controller` is also driving this
   vehicle. Both would publish and the articulation would follow whichever arrived last, so the
   mount judders instead of holding. Logs an ERROR naming the topic and the likely cause.
   Deferred rather than checked at startup because the two nodes usually start together.

## Dependencies
Imports `GIMBAL_MODELS` and `YAW_JOINT_OFFSET` from `los_rate_controller` rather than restating
them — a duplicated convention constant is exactly the thing that drifts. Publishes
`isaac_joint_commands` consumed by Isaac Sim; `gimbal_state_rpy_deg` and `camera/zoom_level`
consumed by `mas_bearing_loc` (`raw_los_node`, `simple_ekf_node`, `dc_ekf_node`,
`direct_projection_ekf_node`, `bearing_debug_viz`, `bearing_residual_monitor`).

## Key Files
- `gimbal_stabilizer/fixed_mount_publisher.py` — node implementation
- `launch/multi_agent_fixed_mount.launch.py` — per-vehicle fan-out over `config/vehicles.yaml`;
  `namespaces:=px4_1,px4_2` restricts which vehicles get a fixed mount, so a mixed
  fixed/gimballed session can be assembled by launching this for one subset and
  `multi_agent_los_rate*.launch.py` for the complement
- `config/vehicles_gimballed.yaml` — the complement list for a mixed session, passed to
  `multi_agent_los_rate_aggressive.launch.py config_file:=…`. Expressing the complement in a
  new config file avoids editing that launch file, which every RAL cohort depends on. It must
  not overlap `FIXED_CAMERA_VEHICLES`; guard 2 above catches it if it does
- `../../tmux/isaac_sim_fixedcam.tmuxp.yaml` (in `IsaacPX4/tmux`) — the session that wires all
  of this together

## Calling Contract

**Pattern**: Timer-only on the output path; the one subscription is instrumentation.

- `_timer_callback()` (50 Hz): publishes all four topics. Sole periodic mutation point.
- `_joint_state_callback()`: updates the peak-deflection hold and warns. Never publishes and
  never feeds the command — `gimbal_state_rpy_deg` carries the CONSTANT, as the field does and
  as a truly rigid mount would, so the PD drive's transient deflection cannot contaminate the
  estimator input. It is reported out of band instead.

## Sign Chain
The USD joint signs are inverted with respect to the controller-internal convention, and the
yaw joint carries a mesh offset. Verified across three files (ticket 055 `r_research.md` §5.1):

| | internal | USD joint |
|---|---|---|
| yaw | 0 (forward) | `yaw_joint = +π/2` (`YAW_JOINT_OFFSET`) |
| roll | `roll` | `roll_joint = −roll` |
| pitch | `pitch` (**positive = DOWN**) | `pitch_joint = −pitch` |

So a mount pitched **UP** by 30.8° is internal pitch −30.8° ⇒ `gimbal_state_rpy_deg.y = −30.8`
(byte-identical to the field shim) and `pitch_joint = +30.8° = +0.5376 rad`.

**The yaw offset is the easy mistake**: at `yaw_joint = 0` the camera looks along body −Y — out
the right side — so commanding pitch alone points the camera at the vehicle's flank.
