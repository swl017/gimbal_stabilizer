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
World-frame LOS (Line-of-Sight) rate gimbal controller for RL policy deployment testing. Ports the iris_ma6 analytical gimbal controller to ROS2. Accepts normalized azimuth/elevation rate commands (matching the RL policy action space), integrates them in world frame, and computes body-frame joint positions via analytical inverse kinematics.

## Subscriptions
- `gimbal_cmd_los_rate` (`geometry_msgs/msg/Vector3`) — normalized LOS rate commands: x=azimuth_rate, y=elevation_rate [-1,1]
- `mavros/imu/data` (`sensor_msgs/msg/Imu`) — vehicle IMU data (quaternion for IK)
- `isaac_joint_states` (`sensor_msgs/msg/JointState`) — actual joint positions for feedback blend

## Publishers
- `isaac_joint_commands` (`sensor_msgs/msg/JointState`) — joint position targets [yaw+offset, roll, pitch]
- `gimbal_state_rpy_rad` (`geometry_msgs/msg/Vector3`) — body-frame RPY in radians
- `gimbal_state_rpy_deg` (`geometry_msgs/msg/Vector3`) — body-frame RPY in degrees
- `gimbal_los_state` (`geometry_msgs/msg/Vector3`) — world-frame azimuth (x) / elevation (y) in radians

## Parameters
- `max_gimbal_rate` (`float`, default: `6.283185`) — maximum gimbal rate [rad/s] (2π = 360 deg/s)
- `yaw_limits_deg` (`list[float]`, default: `[-160, 160]`) — yaw joint limits [deg]
- `pitch_limits_deg` (`list[float]`, default: `[-45, 45]`) — pitch joint limits [deg]
- `roll_limits_deg` (`list[float]`, default: `[-45, 45]`) — roll joint limits [deg]
- `feedback_blend` (`float`, default: `0.05`) — drift correction factor (0.0 for direct state writing)
- `update_rate` (`float`, default: `100.0`) — control loop rate [Hz]

## Dependencies
Subscribes to `mavros/imu/data` published by offboard_py. Subscribes to `isaac_joint_states` from Isaac Sim. Publishes `isaac_joint_commands` consumed by Isaac Sim. **Mutually exclusive** with gimbal_stabilizer — only one should run per vehicle.

## Key Files
- `gimbal_stabilizer/los_rate_controller.py` — Node implementation
- `config/los_rate_config.yaml` — Default parameters
- `launch/multi_agent_los_rate.launch.py` — Multi-agent launch file

## Calling Contract

**Pattern**: Decoupled (subscribe → cache, timer → publish)

- `_los_rate_cmd_callback()`: Caches normalized azimuth/elevation rate commands. No publishing.
- `_imu_callback()`: Caches vehicle quaternion (converts xyzw→wxyz). No publishing.
- `_joint_state_callback()`: Caches actual joint positions by name. No publishing.
- `_timer_callback()` (100 Hz): Feedback blend → rate integration → world-to-body IK → stabilizing roll → clamp → publish. Sole periodic mutation point.

## Joint Mapping
Same as gimbal_stabilizer:
- `cgo3_vertical_arm_joint` — yaw (YAW_JOINT_OFFSET = -π/2 applied at publish)
- `cgo3_horizontal_arm_joint` — roll
- `cgo3_camera_joint` — pitch

## Reference Implementation
Ported from `IsaacLab/.../iris_ma6/controller/gimbal_controller_analytical.py`

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
