# gimbal_stabilizer Architecture

Gimbal camera stabilization control. Two mutually exclusive controllers:
1. **gimbal_stabilizer**: Body-frame RPY position-based stabilization (counteracts vehicle rotation)
2. **los_rate_controller**: World-frame LOS rate control for RL policy deployment (iris_ma6 port)

Only one controller should run per vehicle — they both publish to `isaac_joint_commands`.

## Node Graph

```
┌──────────────────────────────────────────────────────────┐
│             multi_agent_gimbal.launch.py                 │
│  Spawns one node per vehicle from config/vehicles.yaml   │
└──────────────────┬───────────────────────────────────────┘
                   │ launches (per vehicle namespace)
    ┌──────────────┼──────────────────────────────────┐
    │              │                                    │
    ▼              ▼                                    ▼
 gimbal_stabilizer  los_rate_controller     joint_state_publisher
 (body-frame RPY)   (world-frame LOS rate)  (test/example)
     [mutually exclusive — pick one]
```

**LOS rate controller** has its own launch file: `multi_agent_los_rate.launch.py`

## Directed Dependencies

```
# gimbal_stabilizer (body-frame RPY mode)
Isaac Sim ──[isaac_joint_states]──→ gimbal_stabilizer       (JointState: current gimbal positions)
offboard_py ──[mavros/imu/data]──→ gimbal_stabilizer        (Imu: vehicle orientation for stabilization)
offboard_py ──[state/pose]──→ gimbal_stabilizer             (PoseStamped: vehicle pose)
external ──[gimbal_command_rpy_deg]──→ gimbal_stabilizer    (Vector3: desired gimbal angles)
gimbal_stabilizer ──[isaac_joint_commands]──→ Isaac Sim     (JointState: joint position commands)
gimbal_stabilizer ──[gimbal_state_rpy_rad]──→ downstream    (Vector3: current gimbal RPY)
gimbal_stabilizer ──[gimbal_state_rpy_deg]──→ downstream    (Vector3: current gimbal RPY in degrees)

# los_rate_controller (world-frame LOS rate mode)
Isaac Sim ──[isaac_joint_states]──→ los_rate_controller     (JointState: feedback blend)
offboard_py ──[mavros/imu/data]──→ los_rate_controller      (Imu: vehicle quaternion for IK)
RL policy ──[gimbal_cmd_los_rate]──→ los_rate_controller    (Vector3: normalized az/el rates)
los_rate_controller ──[isaac_joint_commands]──→ Isaac Sim   (JointState: joint position commands)
los_rate_controller ──[gimbal_state_rpy_rad]──→ downstream  (Vector3: body-frame RPY)
los_rate_controller ──[gimbal_state_rpy_deg]──→ downstream  (Vector3: body-frame RPY degrees)
los_rate_controller ──[gimbal_los_state]──→ downstream      (Vector3: world-frame az/el)
```

The launch file is the sole integration point. Nodes should be independently testable.

## Data Flow

```
gimbal_stabilizer (Decoupled pattern):
  Subscriber Callbacks (isaac_joint_states, mavros/imu/data, state/pose, gimbal_command_rpy_deg)
    ├─ Cache joint positions, vehicle orientation, gimbal commands
         │
         ▼
  Timer Callback (100 Hz)
    ├─ Compute stabilized joint angles (counteract vehicle rotation)
    ├─ Apply gimbal command offsets
    ├─ Publish isaac_joint_commands
    ├─ Publish gimbal_state_rpy_rad, gimbal_state_rpy_deg

los_rate_controller (Decoupled pattern):
  Subscriber Callbacks (gimbal_cmd_los_rate, mavros/imu/data, isaac_joint_states)
    ├─ Cache LOS rate commands, vehicle quaternion, actual joint positions
         │
         ▼
  Timer Callback (100 Hz)
    ├─ Feedback blend (correct state drift from actual joints)
    ├─ Integrate world-frame azimuth/elevation rates
    ├─ World-to-body IK (atan2 decomposition via quaternion inverse rotation)
    ├─ Compute stabilizing roll (horizon leveling)
    ├─ Clamp to joint limits
    ├─ Publish isaac_joint_commands, gimbal_state, gimbal_los_state
```

## Topic/Service Interface

| Name | Msg/Srv Type | Direction | QoS | Node | Description |
|------|-------------|-----------|-----|------|-------------|
| `isaac_joint_states` | `sensor_msgs/JointState` | Sub | BestEffort | both | Current gimbal joint positions |
| `state/pose` | `geometry_msgs/PoseStamped` | Sub | BestEffort | gimbal_stabilizer | Vehicle pose |
| `mavros/imu/data` | `sensor_msgs/Imu` | Sub | BestEffort | both | Vehicle IMU orientation |
| `gimbal_command_rpy_deg` | `geometry_msgs/Vector3` | Sub | BestEffort | gimbal_stabilizer | Desired gimbal angles (degrees) |
| `gimbal_cmd_los_rate` | `geometry_msgs/Vector3` | Sub | BestEffort | los_rate_controller | Normalized az/el rates [-1,1] |
| `isaac_joint_commands` | `sensor_msgs/JointState` | Pub | Reliable | both | Joint position commands |
| `gimbal_state_rpy_rad` | `geometry_msgs/Vector3` | Pub | BestEffort | both | Body-frame RPY (radians) |
| `gimbal_state_rpy_deg` | `geometry_msgs/Vector3` | Pub | BestEffort | both | Body-frame RPY (degrees) |
| `gimbal_los_state` | `geometry_msgs/Vector3` | Pub | BestEffort | los_rate_controller | World-frame az/el (radians) |

## Parameters

| Parameter | Type | Default | Node | Description |
|-----------|------|---------|------|-------------|
| `gimbal_angle_order` | string | `zyx` | gimbal_stabilizer | Euler angle convention (zyx or zxy) |
| `mounting_orientation` | dict | — | gimbal_stabilizer | Roll/pitch/yaw mounting offsets |
| `joint_limits` | dict | — | gimbal_stabilizer | Min/max angles and max rates per joint |
| `filter` | dict | — | gimbal_stabilizer | Low-pass filter settings (0.1 Hz default) |
| `max_gimbal_rate` | float | 6.283 (2π) | los_rate_controller | Maximum gimbal rate [rad/s] |
| `yaw_limits_deg` | list | [-160, 160] | los_rate_controller | Yaw joint limits [deg] |
| `pitch_limits_deg` | list | [-45, 45] | los_rate_controller | Pitch joint limits [deg] |
| `roll_limits_deg` | list | [-45, 45] | los_rate_controller | Roll joint limits [deg] |
| `feedback_blend` | float | 0.05 | los_rate_controller | Drift correction blend factor |
| `update_rate` | float | 100.0 | both | Control loop rate [Hz] |

## Node Isolation

**Has dependencies** (connected via topics/services):
- `gimbal_stabilizer` — subscribes to Isaac Sim joint states, vehicle IMU, and gimbal commands; publishes joint commands back to Isaac Sim
- `los_rate_controller` — subscribes to LOS rate commands, vehicle IMU, and joint states; publishes joint commands back to Isaac Sim
- `joint_state_publisher` — standalone test node, publishes fixed joint angles

## Stateful Mutation Rule

Both `gimbal_stabilizer` and `los_rate_controller` use the **decoupled pattern**: subscriber callbacks cache incoming data, the timer callback is the sole periodic computation + publish point. Single-threaded executor.

`joint_state_publisher` is stateless — publishes fixed values on a timer.

## File Conventions

- `gimbal_stabilizer/*.py` — Node implementations
- `config/gimbal_config.yaml` — Gimbal stabilizer parameters
- `config/los_rate_config.yaml` — LOS rate controller parameters
- `config/vehicles.yaml` — Multi-agent vehicle configuration
- `launch/multi_agent_gimbal.launch.py` — Multi-agent launch (gimbal_stabilizer mode)
- `launch/multi_agent_los_rate.launch.py` — Multi-agent launch (los_rate_controller mode)
- `CONTEXT.md` — Node routing contracts
- `doc/*_spec.md` — Authoritative specifications
