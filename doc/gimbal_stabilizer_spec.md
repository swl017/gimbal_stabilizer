# gimbal_stabilizer Specification

**Module:** `gimbal_stabilizer`
**Status:** Draft v1
**Last Updated:** 2026-03-24

---

## 1. Motivation and Design Philosophy

### 1.1 The Problem
The current ROS2 implementation of gimbal stabilizer is crude and outdated. We would like to match the architecture from `iris_ma6` project.

### 1.2 Design Philosophy
We are building SITL simulation as part of sim-to-real deployment using policy trained in Isaac Lab. The policy was trained with gimbal stabilizer in the Isaac Lab environment. The controller helped keeping the world-frame LOS when no(zero) LOS-rate command(azimuth/elevation rate) was applied. The objective is to implement the (architecturally) same controller to SITL environment, so as a result, the policy training becomes a *Software-in-the-Learning-Loop* fashion.

### 1.3 Architecture Overview
```
                    RL Policy (or manual command)
                              │
                    gimbal_cmd_los_rate
                    [az_rate, el_rate] ∈ [-1,1]
                              │
                              ▼
              ┌───────────────────────────────┐
              │       LOSRateController       │
              │  (ROS2 Node, 100 Hz timer)    │
              │                               │
              │  1. Feedback blend            │◄──── isaac_joint_states
              │  2. World-frame rate integ.   │      [actual joint pos]
              │  3. World→Body IK (atan2)     │◄──── mavros/imu/data
              │  4. Stabilizing roll          │      [vehicle quaternion]
              │  5. Joint limit clamping      │
              │  6. Publish                   │
              └───────────────┬───────────────┘
                              │
                    isaac_joint_commands
                    JointState [pos, vel]
                              │
           ┌──────────────────┼──────────────────┐
           │  PegasusSimulator OmniGraph          │
           │  (per-vehicle ActionGraph)            │
           │                                      │
           │  ROS2SubscribeJointState              │
           │          │                            │
           │          ▼                            │
           │  IsaacArticulationController          │
           │          │                            │
           │          ▼                            │
           │  PhysX joints (yaw/roll/pitch)        │
           │          │                            │
           │          ▼                            │
           │  ROS2PublishJointState ───────────────┼──► isaac_joint_states
           └──────────────────────────────────────┘
```

The OmniGraph ActionGraph is configured per vehicle in the PegasusSimulator launch script (e.g. `px4_multi_world_iris_gimbal3.isaac.py`). It wires `ROS2SubscribeJointState` → `IsaacArticulationController` → `ROS2PublishJointState`, providing both command application and state feedback via ROS2 topics.

## 2. Mathematical Formulation

### 2.1 System Model

**Notation:**

| Symbol | Description | Frame |
|--------|-------------|-------|
| $\alpha$ | Azimuth (world-frame yaw of LOS) | World (ENU) |
| $\epsilon$ | Elevation (world-frame pitch of LOS) | World (ENU) |
| $\dot{\alpha}_{cmd}$, $\dot{\epsilon}_{cmd}$ | Normalized rate commands ∈ [-1, 1] | — |
| $\Omega_{max}$ | Max gimbal rate [rad/s] | — |
| $\mathbf{q}_{body}$ | Vehicle quaternion (wxyz) | World→Body |
| $\psi$, $\phi$, $\theta$ | Gimbal yaw, roll, pitch joint angles | Body (FLU) |
| $\beta$ | Feedback blend factor | — |
| $\Delta t$ | Control timestep (1/update_rate) | — |

**Joint order:** Yaw(Z) → Roll(X) → Pitch(Y), with `YAW_JOINT_OFFSET = -π/2` applied at the joint command interface (compensates 90° body mesh visual rotation).

**Quaternion convention:** Internal math uses wxyz (matching Isaac Lab). ROS2 Imu messages arrive as xyzw and are converted on receipt.

### 2.2 Algorithm Details

The control loop runs at `update_rate` Hz (default 100 Hz). Each tick:

**Step 1 — Feedback blend** (drift correction from actual joints):
```
ψ ← ψ + β · (ψ_actual - YAW_JOINT_OFFSET - ψ)
φ ← φ + β · (φ_actual - φ)
θ ← θ + β · (θ_actual - θ)
```

**Step 2 — World-frame rate integration:**
```
α ← α + (α̇_cmd · Ω_max) · Δt          wrap to [-π, π]
ε ← ε + (ε̇_cmd · Ω_max) · Δt          clamp to pitch_limits
```
When rate commands are zero, α and ε are constant in the world frame — the body-frame decomposition (Step 3) automatically counter-rotates to maintain LOS.

**Step 3 — World-to-body inverse kinematics** (analytical atan2 decomposition):
```
d_world = [cos(ε)·cos(α),  cos(ε)·sin(α),  sin(ε)]
d_body  = R(q_body)⁻¹ · d_world

ψ_new = atan2(d_body_y, d_body_x)
θ_new = -atan2(d_body_z, √(d_body_x² + d_body_y²))
```

**Step 4 — Stabilizing roll** (horizon leveling, independent of LOS):
```
u_body = R(q_body)⁻¹ · [0, 0, 1]        (world-up in body frame)
u_y_yawed = -u_body_x · sin(ψ) + u_body_y · cos(ψ)
u_z_yawed =  u_body_z

φ_new = atan2(-u_y_yawed, u_z_yawed)
```

**Step 5 — Update state:**
```
ψ ← ψ_new,   θ ← θ_new,   φ ← φ_new
```

**Step 6 — Joint limit clamping:**
```
ψ ← clamp(ψ, yaw_limits)
θ ← clamp(θ, pitch_limits)
φ ← clamp(φ, roll_limits)
```

**Step 7 — Publish** (mode-dependent, see §3.1):

| Mode | Position field | Velocity field |
|------|---------------|----------------|
| `position` | `[ψ + OFFSET, φ, θ]` | *(empty)* |
| `position_velocity` | `[ψ + OFFSET, φ, θ]` | `[ψ̇, φ̇, θ̇]` via finite diff |
| `velocity` | *(empty)* | `[ψ̇, φ̇, θ̇]` via finite diff |

Velocity targets (finite difference):
```
ψ̇ = (ψ - ψ_prev) / Δt
φ̇ = (φ - φ_prev) / Δt
θ̇ = (θ - θ_prev) / Δt
```

## 3. Implementation Details

### 3.1 Configuration

#### Vehicle and Model Configuration

Source: `config/vehicles.yaml`

Each vehicle entry specifies the gimbal model, which determines joint names and offsets:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `namespace` | str | — | ROS2 namespace for multi-agent (e.g. `"px4_1"`) |
| `update_rate` | float | 100.0 | Control loop frequency [Hz] |
| `model` | str | `"cgo3"` | Gimbal model name. Selects joint name mapping. |

**Supported models and their joint names:**

| Model | Yaw Joint | Roll Joint | Pitch Joint | USD Asset(s) |
|-------|-----------|------------|-------------|--------------|
| `cgo3` | `cgo3_vertical_arm_joint` | `cgo3_horizontal_arm_joint` | `cgo3_camera_joint` | `iris_gimbal.usda`, `typhoon_h480.usda` |
| `iris_gimbal3` | `yaw_joint` | `roll_joint` | `pitch_joint` | `iris_gimbal3.usda`, `iris_gimbal2.usda` |

USD assets live under `PegasusSimulator/.../assets/Robots/`. The model name is used to look up joint names at node startup. Joint names are published in `isaac_joint_commands.name` and matched against `isaac_joint_states.name` for feedback blend.

#### Controller Configuration

Source: `config/los_rate_config.yaml`

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `max_gimbal_rate` | float | 6.283185 (2π) | Max angular rate [rad/s] (360 deg/s) |
| `yaw_limits_deg` | [float, float] | [-160.0, 160.0] | Yaw joint limits [deg] |
| `pitch_limits_deg` | [float, float] | [-45.0, 45.0] | Pitch joint limits [deg]. Limited to avoid gimbal lock at ±90°. |
| `roll_limits_deg` | [float, float] | [-45.0, 45.0] | Roll joint limits [deg] |
| `feedback_blend` | float | 0.05 | Drift correction factor β. 0.0 = open-loop (direct state setting). 0.05–0.2 = closed-loop (implicit actuator with PD lag). |
| `control_mode` | str | `"position"` | Actuator command mode: `"position"`, `"position_velocity"`, or `"velocity"` |

**Tuning guidance:**
- `feedback_blend`: Use 0.0 when actuator applies joint state directly (no PD). Use 0.05 with implicit PD actuator (k=1000, d=50) to correct ~50 ms servo lag.
- `control_mode`: Use `"position_velocity"` to match iris_ma6's implicit actuator (PD with velocity feedforward). Use `"position"` for simplicity. Use `"velocity"` for pure rate control without position tracking.

### 3.2 Core Class

**`LOSRateController(Node)`** — ROS2 node implementing the analytical gimbal controller from iris_ma6.

**Internal state** (persists across timer ticks):

| Field | Type | Init | Description |
|-------|------|------|-------------|
| `_azimuth_world` | float | 0.0 | World-frame LOS azimuth [rad] |
| `_elevation_world` | float | 0.0 | World-frame LOS elevation [rad] |
| `_yaw` | float | 0.0 | Body-frame yaw joint target [rad] |
| `_roll` | float | 0.0 | Body-frame roll joint target [rad] |
| `_pitch` | float | 0.0 | Body-frame pitch joint target [rad] |
| `_joint_names` | list[str] | *(from model)* | Joint names resolved from `model` config |

**Key methods:**

| Method | Role |
|--------|------|
| `_timer_callback()` | Main control loop (Steps 1–7). Sole writer of internal state. |
| `_imu_callback(msg)` | Caches vehicle quaternion (xyzw → wxyz conversion). |
| `_joint_state_callback(msg)` | Caches actual joint positions by name (uses `_joint_names` for matching). |
| `_los_rate_cmd_callback(msg)` | Caches normalized rate commands (clips to [-1, 1]). |
| `_world_to_body_angles(α, ε, q)` | Static. Analytical IK: world pointing → body yaw/pitch. |
| `_compute_stabilizing_roll(ψ, q)` | Static. Projects world-up into yawed gimbal frame → roll. |
| `_publish_joint_commands()` | Publishes JointState with YAW_JOINT_OFFSET applied. Uses `_joint_names`. |
| `_publish_state()` | Publishes RPY (rad/deg) and world-frame LOS state. |

**Helper function:**

`_quat_rotate_inverse(q_wxyz, v)` — Rotates vector v by conjugate of quaternion q using the cross-product form: `v' = v + 2w(u×v) + 2(u×(u×v))` where `u = -[x,y,z]`.

## 4. Integration Points

### 4.1 Input Interface

| Input | Topic | Msg Type | Field(s) | Range | Source |
|-------|-------|----------|----------|-------|--------|
| LOS rate command | `gimbal_cmd_los_rate` | `Vector3` | x=az_rate, y=el_rate | [-1, 1] normalized | RL policy / manual |
| Vehicle orientation | `mavros/imu/data` | `Imu` | orientation (xyzw) | unit quaternion | offboard_py (mavros bridge) |
| Actual joint positions | `isaac_joint_states` | `JointState` | name + position | [rad] | Isaac Sim |

### 4.2 Output Interface

| Output | Topic | Msg Type | Field(s) | Range | Consumer |
|--------|-------|----------|----------|-------|----------|
| Joint commands | `isaac_joint_commands` | `JointState` | name (model-dependent), position=[ψ+offset, φ, θ], velocity=[ψ̇, φ̇, θ̇] | [rad], [rad/s] | Isaac Sim |
| Gimbal state (rad) | `gimbal_state_rpy_rad` | `Vector3` | x=roll, y=pitch, z=yaw | [rad] | Monitoring |
| Gimbal state (deg) | `gimbal_state_rpy_deg` | `Vector3` | x=roll, y=pitch, z=yaw | [deg] | Monitoring |
| World LOS state | `gimbal_los_state_deg` | `Vector3` | x=azimuth, y=elevation | [deg] | Monitoring |

### 4.3 Dependencies

**Upstream:**
- **offboard_py** — Bridges PX4 IMU to `mavros/imu/data` topic
- **RL policy** (or manual publisher) — Publishes rate commands on `gimbal_cmd_los_rate`
- **Isaac Sim** — Publishes actual joint state on `isaac_joint_states`

**Downstream:**
- **Isaac Sim** — Consumes joint commands from `isaac_joint_commands`

**Multi-agent:** Each vehicle gets an independent `LOSRateController` instance, namespaced via launch file (e.g., `/px4_1/los_rate_controller`). The `model` parameter is set per-vehicle in `vehicles.yaml`.

### 4.4 Calling Contract

| Method | Type | Frequency | Lifecycle Hook | Notes |
|--------|------|-----------|---------------|-------|
| `_timer_callback` | WRITE | 100 Hz (configurable) | ROS2 timer | Sole writer of internal state. Skips if no IMU received. |
| `_imu_callback` | READ | Async (sensor rate) | ROS2 subscriber | Caches quaternion. Safe for multiple calls. |
| `_joint_state_callback` | READ | Async (sim rate) | ROS2 subscriber | Caches joint positions. Matches by `_joint_names`. |
| `_los_rate_cmd_callback` | READ | Async (policy rate) | ROS2 subscriber | Caches rate command. Clips to [-1, 1]. |

**Stateful invariants:**
- **Single-threaded executor**: The node uses ROS2's default single-threaded executor, so callbacks are serialized. No mutex needed.
- **Timer is sole writer**: Only `_timer_callback` mutates `_azimuth_world`, `_elevation_world`, `_yaw`, `_roll`, `_pitch`. Subscriber callbacks only write to cached input fields.
- **Graceful degradation**: If IMU has not been received (`_vehicle_quat_wxyz is None`), the timer callback returns immediately without publishing.
- **Feedback blend guard**: Feedback blend is skipped if `_joint_positions_actual is None` (no joint state received yet) or `feedback_blend == 0.0`.

## 5. Validation and Testing

### 5.1 Unit Tests

| Category | Test | Pass Criteria |
|----------|------|---------------|
| IK correctness | World pointing (0,0) with identity q_body → yaw=0, pitch=0 | Exact match |
| IK correctness | World pointing (π/4, 0) with identity q_body → yaw=π/4 | atol < 1e-6 |
| IK correctness | World pointing (0,0) with 30° pitched body → body yaw compensates | LOS error < 0.1° |
| Stabilizing roll | Level body → roll ≈ 0 | atol < 1e-6 |
| Stabilizing roll | 30° rolled body → roll ≈ -30° (counter-rotation) | atol < 1° |
| Joint limits | Command beyond limits → clamped to configured range | Exact clamp |
| Feedback blend | β=0.05, 100 steps with 10° offset → converges within 5% | Exponential decay |
| Rate integration | Zero command for 100 steps → α, ε unchanged | Exact match |
| Rate integration | Max command for 1 step → Δα = Ω_max · Δt | atol < 1e-6 |
| Velocity output | Finite difference matches position delta / dt | atol < 1e-4 |
| Joint name mapping | Model `"cgo3"` → correct CGO3 joint names | Exact match |
| Joint name mapping | Model `"iris_gimbal3"` → correct iris_gimbal3 joint names | Exact match |

### 5.2 Integration Tests

| Test | Setup | Pass Criteria |
|------|-------|---------------|
| Hover stabilization | Isaac Sim + offboard_py, drone hovering, zero rate cmd | LOS drift < 0.5° over 10s |
| Pitch maneuver | Drone pitches 30° forward, zero rate cmd | LOS error < 2° RMS |
| Yaw tracking | Constant az_rate = 0.5 while hovering | Smooth yaw sweep, no oscillation |
| Multi-agent | 3 drones, independent controllers | No cross-talk between namespaces |
| Mode switching | Switch control_mode at runtime | Smooth transition, no joint jumps |
| Model selection | Launch with `model: iris_gimbal3` | Correct joint names in published JointState |

## 6. Known Limitations

- **Cross-axis error at high tilt**: During aggressive maneuvers (>30° body tilt), roll joint saturates at ±45°, leaving only ~8° margin. Cross-axis LOS error can reach 7–14° RMS. This is a physical joint range constraint, not a controller bug.
- **Analytical mode only**: No Jacobian mode with velocity feedforward tracking (unlike iris_ma6 which supports both). The analytical mode is zero-lag and requires no gain tuning.
- **Single-vehicle per node**: Each node handles one vehicle. No batched/vectorized processing (unlike iris_ma6's GPU-tensorized controller).
- **No gimbal lock protection**: Pitch limits are set to ±45° to stay away from the ±90° singularity, but no explicit singularity detection is implemented.
- **OmniGraph dependency**: Gimbal command reception relies on an OmniGraph ActionGraph configured in the PegasusSimulator launch script, not PegasusSimulator's Python `ROS2Backend`. If a launch script omits the ActionGraph setup, gimbal commands will not be applied.

## 7. References
- **`iris_ma6` gimbal controller** `IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/controller/CONTEXT.md`
- **Analytical controller source** `IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/controller/gimbal_controller_analytical.py`
- **Controller config** `IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/controller/gimbal_controller_cfg.py`
- **ROS2 los_rate_controller** `ros2_ws/src/gimbal_stabilizer/gimbal_stabilizer/los_rate_controller.py`
- **ROS2 config** `ros2_ws/src/gimbal_stabilizer/config/los_rate_config.yaml`
- **Actuator config (iris_gimbal3)** `IsaacLab/source/isaaclab_assets/isaaclab_assets/robots/iris_gimbal3.py` — ImplicitActuatorCfg: k=1000, d=50, vel_limit=6π rad/s
- **Gimbal stabilization tuning results** `IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/controller/doc/gimbal_stabilization_status.md`
- **PegasusSimulator SITL launch (OmniGraph setup)** `PegasusSimulator/launch/px4_multi_world_iris_gimbal3.isaac.py`
