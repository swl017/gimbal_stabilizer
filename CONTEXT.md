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
- `gimbal_cmd_los_rate` (`geometry_msgs/msg/Vector3`) — normalized LOS rate commands: x=azimuth_rate, y=elevation_rate [-1,1] (rate mode)
- `gimbal_cmd_los_world_deg` (`geometry_msgs/msg/Vector3`) — world-frame position target: z=azimuth, y=elevation [deg] (position mode)
- `mavros/imu/data` (`sensor_msgs/msg/Imu`) — vehicle IMU (quaternion + angular velocity for stabilization)
- `isaac_joint_states` (`sensor_msgs/msg/JointState`) — actual joint positions (feedback)
- `camera/zoom` (`std_msgs/msg/Float64`) — sim camera zoom level
- `zoom_cmd` (`std_msgs/msg/Float32`) — normalized zoom rate command [-1,1]

## Publishers
- `isaac_joint_commands` (`sensor_msgs/msg/JointState`) — joint targets (position and/or velocity)
- `gimbal_state_rpy_deg` (`geometry_msgs/msg/Vector3`) — actual body-frame RPY in degrees (from joint feedback)
- `gimbal_los_state_deg` (`geometry_msgs/msg/Vector3`) — world-frame azimuth (x) / elevation (y) in degrees
- `combined_ang_vel_w` (`geometry_msgs/msg/Vector3Stamped`) — body + gimbal angular velocity in world frame
- `zoom_level` (`std_msgs/msg/Float32`) — current zoom level
- `camera/zoom` (`std_msgs/msg/Float64`) — sim camera zoom command

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
