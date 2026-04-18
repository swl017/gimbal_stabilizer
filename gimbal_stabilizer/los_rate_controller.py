#!/usr/bin/env python3
from __future__ import annotations
"""World-frame LOS-rate gimbal controller for RL policy deployment.

Ports the iris_ma6 Jacobian gimbal controller to ROS2. Accepts normalized
world-frame azimuth/elevation rate commands, integrates them into a
persistent world-frame target, and computes body-frame joint positions
via the unified Jacobian-inverse control law.

Rate mode algorithm (from iris_ma6 gimbal_controller_jacobian.py):
  1. Integrate LOS rate commands into world-frame az/el target
  2. Compute desired camera quaternion in body frame
  3. Quaternion error from actual joints → body angular error
  4. omega_cmd = -K_pointing * att_error
  5. qdot = J^{-1} * (omega_cmd - omega_body)
  6. Position target = actual + qdot * dt

Position mode algorithm (analytical IK, for point_to_region):
  1. World-frame target → body-frame yaw/pitch via atan2 IK
  2. Stabilizing roll from world-up projection
  3. Direct position targets (no velocity dynamics)

LOS convention: positive elevation = up (ENU standard).

Frame Conventions:
- WORLD (ENU): +X=East, +Y=North, +Z=Up
- BODY (FLU):  +X=Forward, +Y=Left, +Z=Up
- Gimbal joint order: Yaw(Z) -> Roll(X) -> Pitch(Y)
- YAW_JOINT_OFFSET = +pi/2 (body mesh visual rotation compensation)

Quaternion Convention:
- ROS2 Imu msg: xyzw
- Internal math: wxyz (matching Isaac Lab / iris_ma6)

Gimbal Jacobian (maps joint velocities -> body-frame angular velocity):
  J = [[ 0,      cos(psi),    -sin(psi)*cos(phi) ],
       [ 0,      sin(psi),     cos(psi)*cos(phi) ],
       [ 1,      0,            sin(phi)           ]]

  J^{-1} = [[ sy*sr/cr,  -cy*sr/cr,  1 ],
            [ cy,         sy,         0 ],
            [-sy/cr,      cy/cr,      0 ]]
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3, Vector3Stamped
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32, Float64, Header

# With identity camera orientation on pitch_link, yaw_joint=0 points camera
# along body -Y (right side). This offset is added to yaw joint targets so
# that controller yaw=0 maps to body +X (physics forward).
YAW_JOINT_OFFSET = math.pi / 2

# Joint name mapping per gimbal model.  Order: [yaw, roll, pitch].
GIMBAL_MODELS: dict[str, list[str]] = {
    'cgo3': [
        'cgo3_vertical_arm_joint',     # yaw
        'cgo3_horizontal_arm_joint',   # roll
        'cgo3_camera_joint',           # pitch
    ],
    'iris_gimbal3': [
        'yaw_joint',                   # yaw
        'roll_joint',                  # roll
        'pitch_joint',                 # pitch
    ],
}


# ── Quaternion helpers (wxyz convention) ─────────────────────────────

def _quat_rotate_inverse(q_wxyz: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate vector v by the inverse of quaternion q (wxyz format)."""
    w = q_wxyz[0]
    u = -q_wxyz[1:4]  # conjugate
    t = 2.0 * np.cross(u, v)
    return v + w * t + np.cross(u, t)


def _quat_rotate(q_wxyz: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate vector v by quaternion q (wxyz format)."""
    w = q_wxyz[0]
    u = q_wxyz[1:4]
    t = 2.0 * np.cross(u, v)
    return v + w * t + np.cross(u, t)


def _quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Multiply two quaternions in wxyz format."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def _quat_inv(q: np.ndarray) -> np.ndarray:
    """Inverse (conjugate) of unit quaternion in wxyz format."""
    return np.array([q[0], -q[1], -q[2], -q[3]])


def _quat_from_euler_single(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Euler XYZ (roll, pitch, yaw) → quaternion wxyz."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


def _rotmat_to_quat(R: np.ndarray) -> np.ndarray:
    """Rotation matrix → quaternion wxyz (Shepperd's method)."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 2.0 * math.sqrt(1.0 + trace)
        return np.array([s / 4, (R[2, 1] - R[1, 2]) / s,
                         (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s])
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        return np.array([(R[2, 1] - R[1, 2]) / s, s / 4,
                         (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s])
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 - R[0, 0] + R[1, 1] - R[2, 2])
        return np.array([(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s,
                         s / 4, (R[1, 2] + R[2, 1]) / s])
    else:
        s = 2.0 * math.sqrt(1.0 - R[0, 0] - R[1, 1] + R[2, 2])
        return np.array([(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s,
                         (R[1, 2] + R[2, 1]) / s, s / 4])


# ── Controller node ──────────────────────────────────────────────────

class LOSRateController(Node):
    """World-frame LOS gimbal controller with Jacobian-inverse tracking.

    Rate mode: Jacobian J^{-1} control law (matching iris_ma6 training).
    Position mode: analytical atan2 IK (for point_to_region commands).
    """

    def __init__(self):
        super().__init__('los_rate_controller')

        # -- Declare parameters --
        self.declare_parameter('max_gimbal_rate', math.pi)
        self.declare_parameter('pointing_gain', 32.5)
        self.declare_parameter('yaw_limits_deg', [-45.0, 45.0])
        self.declare_parameter('pitch_limits_deg', [-45.0, 45.0])
        self.declare_parameter('roll_limits_deg', [-45.0, 45.0])
        self.declare_parameter('update_rate', 250.0)
        self.declare_parameter('model', 'iris_gimbal3')
        self.declare_parameter('control_mode', 'position')
        self.declare_parameter('control_trigger', 'joint_states')  # "joint_states", "imu", or "clock"
        self.declare_parameter('zoom_min', 1.0)
        self.declare_parameter('zoom_max', 6.0)
        self.declare_parameter('servo_rate_limit', 18*math.pi)  # 0 = use max_gimbal_rate
        self.declare_parameter('gimbal_controller_mode', 'analytical')  # "analytical" or "jacobian"

        self._max_gimbal_rate = self.get_parameter('max_gimbal_rate').value
        self._pointing_gain = self.get_parameter('pointing_gain').value
        yaw_lim_deg = self.get_parameter('yaw_limits_deg').value
        pitch_lim_deg = self.get_parameter('pitch_limits_deg').value
        roll_lim_deg = self.get_parameter('roll_limits_deg').value
        self._yaw_limits = (math.radians(yaw_lim_deg[0]), math.radians(yaw_lim_deg[1]))
        self._pitch_limits = (math.radians(pitch_lim_deg[0]), math.radians(pitch_lim_deg[1]))
        self._roll_limits = (math.radians(roll_lim_deg[0]), math.radians(roll_lim_deg[1]))
        self._update_rate = float(self.get_parameter('update_rate').value)
        self._control_mode = self.get_parameter('control_mode').value
        self._control_trigger = self.get_parameter('control_trigger').value

        # -- Resolve joint names from model --
        model = self.get_parameter('model').value
        if model not in GIMBAL_MODELS:
            raise ValueError(
                f"Unknown gimbal model '{model}'. "
                f"Supported: {list(GIMBAL_MODELS.keys())}")
        self._joint_names = GIMBAL_MODELS[model]

        # -- Internal state --
        self._yaw = 0.0
        self._roll = 0.0
        self._pitch = 0.0
        # Jacobian-computed joint velocities (used by combined mode)
        self._qdot: np.ndarray | None = None
        # Persistent world-frame LOS target (rate mode integrates into this)
        self._az_world = 0.0
        self._el_world = 0.0
        self._az_world_initialized = False
        # Previous actual joint positions for velocity computation
        self._prev_actual_yaw = 0.0
        self._prev_actual_roll = 0.0
        self._prev_actual_pitch = 0.0
        # Timestamp for actual dt measurement
        self._prev_time: float | None = None
        # Diagnostic logging counter (ticket #022)
        self._diag_counter = 0

        # -- Cached subscriber data --
        self._vehicle_quat_wxyz: np.ndarray | None = None
        self._body_angular_velocity_b: np.ndarray = np.zeros(3)
        self._joint_positions_actual: dict | None = None
        self._cmd_az_rate = 0.0
        self._cmd_el_rate = 0.0
        # World-frame position target (from point_to_region)
        self._target_az_world: float | None = None
        self._target_el_world: float | None = None

        # -- Zoom state --
        self._zoom_level: float = 1.0
        self._zoom_cmd_rate = 0.0
        self._zoom_min = float(self.get_parameter('zoom_min').value or 1.0)
        self._zoom_max = float(self.get_parameter('zoom_max').value or 30.0)
        servo_lim = float(self.get_parameter('servo_rate_limit').value or 6*math.pi)
        self._servo_rate_limit = servo_lim if servo_lim > 0 else self._max_gimbal_rate
        self._gimbal_controller_mode = self.get_parameter('gimbal_controller_mode').value

        # -- QoS profiles --
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=10)

        # -- Publishers --
        self._cmd_pub = self.create_publisher(
            JointState, 'isaac_joint_commands', reliable_qos)
        self._rpy_deg_pub = self.create_publisher(
            Vector3, 'gimbal_state_rpy_deg', sensor_qos)
        self._los_state_pub = self.create_publisher(
            Vector3, 'gimbal_los_state_deg', sensor_qos)
        self._combined_ang_vel_w_pub = self.create_publisher(
            Vector3Stamped, 'combined_ang_vel_w', sensor_qos)
        self._zoom_level_pub = self.create_publisher(
            Float64, 'camera/zoom_level', sensor_qos)
        self._camera_zoom_cmd_pub = self.create_publisher(Float64, 'camera/zoom_level_cmd', 10)

        # -- Subscribers --
        self.create_subscription(
            Vector3, 'gimbal_cmd_los_rate',
            self._los_rate_cmd_callback, sensor_qos)
        self.create_subscription(
            Vector3, 'gimbal_cmd_los_world_deg',
            self._los_world_cmd_callback, reliable_qos)
        self.create_subscription(
            Imu, 'mavros/imu/data',
            self._imu_callback, sensor_qos)
        # isaac_joint_states publishes multiple messages per physics step.
        # Use deeper queue to avoid dropping steps between callback processing.
        joint_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50,
        )
        self.create_subscription(
            JointState, 'isaac_joint_states',
            self._joint_state_callback, joint_qos)
        self.create_subscription(
            Float32, 'zoom_rate_cmd', self._zoom_rate_cmd_callback, 10)
        self.create_subscription(
            Float64, 'zoom_level_set', self._zoom_level_set_callback, 10)
        self.create_subscription(
            Clock, '/clock', self._clock_callback, sensor_qos)

        # No timer — control loop runs in _joint_state_callback,
        # triggered by isaac_joint_states at the sim physics rate.

        self.get_logger().info(
            f'LOS rate controller started: model={model}, '
            f'rate={self._update_rate} Hz, '
            f'max_gimbal_rate={math.degrees(self._max_gimbal_rate):.1f} deg/s, '
            f'pointing_gain={self._pointing_gain}, '
            f'servo_rate_limit={math.degrees(self._servo_rate_limit):.1f} deg/s, '
            f'control_mode={self._control_mode}, '
            f'control_trigger={self._control_trigger}, '
            f'gimbal_controller_mode={self._gimbal_controller_mode}')

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------

    def _los_rate_cmd_callback(self, msg: Vector3):
        """Cache LOS rate commands in rad/s. x=azimuth, y=elevation.

        Positive elevation = up (ENU standard).
        """
        self._cmd_az_rate = float(msg.x)
        self._cmd_el_rate = float(msg.y)
        self._target_az_world = None
        self._target_el_world = None

    def _los_world_cmd_callback(self, msg: Vector3):
        """Cache world-frame azimuth/elevation position target (degrees).

        z=azimuth, y=elevation. Positive elevation = up.
        """
        self._target_az_world = math.radians(msg.z)
        self._target_el_world = math.radians(msg.y)
        self._cmd_az_rate = 0.0
        self._cmd_el_rate = 0.0

    def _imu_callback(self, msg: Imu):
        q = msg.orientation
        self._vehicle_quat_wxyz = np.array([q.w, q.x, q.y, q.z])
        av = msg.angular_velocity
        self._body_angular_velocity_b = np.array([av.x, av.y, av.z])

        if self._control_trigger == 'imu':
            self._run_control_loop_simtime(msg.header.stamp)

    def _clock_callback(self, msg: Clock):
        if self._control_trigger == 'clock' and \
            self._vehicle_quat_wxyz is not None and \
            self._body_angular_velocity_b is not None:
            self._run_control_loop_simtime(msg.clock)

    def _zoom_rate_cmd_callback(self, msg: Float32):
        self._zoom_cmd_rate = float(msg.data)

    def _zoom_level_set_callback(self, msg: Float64):
        """Set zoom level directly (e.g., reset to 1.0 on IDLE)."""
        level = float(msg.data)
        level = max(self._zoom_min, min(self._zoom_max, level))
        self._zoom_level = level
        self._zoom_cmd_rate = 0.0
        # Publish immediately to sim camera
        cam_msg = Float64()
        cam_msg.data = self._zoom_level
        self._camera_zoom_cmd_pub.publish(cam_msg)
        self.get_logger().info(f'Zoom level set to {self._zoom_level:.1f}')

    def _joint_state_callback(self, msg: JointState):
        """Parse and cache joint positions. Run control if trigger='joint_states'."""
        positions = {}
        yaw_name, roll_name, pitch_name = self._joint_names
        for i, name in enumerate(msg.name):
            if name == yaw_name:
                positions['yaw'] = msg.position[i]
            elif name == roll_name:
                positions['roll'] = msg.position[i]
            elif name == pitch_name:
                positions['pitch'] = msg.position[i]
        if len(positions) != 3:
            return
        self._joint_positions_actual = positions

        if self._control_trigger == 'joint_states':
            # Dedup by sim timestamp: skip duplicate messages from same physics step
            stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if self._prev_time is not None and stamp_sec <= self._prev_time:
                return
            if self._vehicle_quat_wxyz is None:
                return
            if self._prev_time is None or stamp_sec <= 0.0:
                self._prev_time = stamp_sec
                dt = 1.0 / self._update_rate
            else:
                dt = stamp_sec - self._prev_time
            self._prev_time = stamp_sec
            self._run_control(dt)

    def _run_control_loop_simtime(self, stamp):
        """Run control loop using sim-time dt from message timestamp.

        dt is clamped to the nominal period (1/update_rate) so that K*dt
        never exceeds the designed value, even when the message source
        runs slower than expected (e.g. PX4 SITL IMU at ~20 Hz instead
        of 250 Hz nominal).
        """
        if self._joint_positions_actual is None:
            return
        stamp_sec = stamp.sec + stamp.nanosec * 1e-9
        if self._prev_time is None or stamp_sec <= 0.0:
            self._prev_time = stamp_sec
            dt = 1.0 / self._update_rate
        else:
            dt = stamp_sec - self._prev_time
            if dt < 1.0 / self._update_rate:
                return
        self._prev_time = stamp_sec
        self._run_control(dt)

    def _run_control(self, dt: float):
        # Read actual joint angles
        # iris_gimbal3 USD joint signs are inverted vs Jacobian/FK convention.
        actual_yaw = self._joint_positions_actual['yaw'] - YAW_JOINT_OFFSET
        actual_roll = -self._joint_positions_actual['roll']
        actual_pitch = -self._joint_positions_actual['pitch']

        if self._target_az_world is not None:
            # ── Position mode (analytical IK for point_to_region) ────
            az_world = self._target_az_world
            el_world = self._target_el_world

            az_world = math.atan2(math.sin(az_world), math.cos(az_world))
            el_world = float(np.clip(
                el_world, self._pitch_limits[0], self._pitch_limits[1]))

            yaw_new, pitch_new = self._world_to_body_angles(
                az_world, el_world, self._vehicle_quat_wxyz)
            roll_new = self._compute_stabilizing_roll(
                yaw_new, self._vehicle_quat_wxyz)

            self._yaw = float(np.clip(yaw_new, self._yaw_limits[0], self._yaw_limits[1]))
            self._pitch = float(np.clip(pitch_new, self._pitch_limits[0], self._pitch_limits[1]))
            self._roll = float(np.clip(roll_new, self._roll_limits[0], self._roll_limits[1]))

        else:
            # ── Rate mode ────────────────────────────────────────────

            # Initialize world-frame target from current joints on first tick
            if not self._az_world_initialized:
                self._az_world, self._el_world = self._body_to_world_angles(
                    actual_yaw, actual_pitch, self._vehicle_quat_wxyz)
                self._az_world_initialized = True

            # 1. Integrate LOS rate into candidate world-frame target
            az_candidate = self._az_world + self._cmd_az_rate * dt
            el_candidate = self._el_world + self._cmd_el_rate * dt
            az_candidate = math.atan2(math.sin(az_candidate), math.cos(az_candidate))
            el_candidate = float(np.clip(
                el_candidate, self._pitch_limits[0], self._pitch_limits[1]))

            # Anti-windup: accept azimuth and elevation independently.
            # If azimuth update would push yaw joint out of limits, reject
            # only the azimuth change. Same for elevation/pitch. This
            # prevents one saturated axis from freezing the other.
            yaw_az, pitch_az = self._world_to_body_angles(
                az_candidate, self._el_world, self._vehicle_quat_wxyz)
            if self._yaw_limits[0] <= yaw_az <= self._yaw_limits[1]:
                self._az_world = az_candidate

            yaw_el, pitch_el = self._world_to_body_angles(
                self._az_world, el_candidate, self._vehicle_quat_wxyz)
            if self._pitch_limits[0] <= pitch_el <= self._pitch_limits[1]:
                self._el_world = el_candidate

            az_world = self._az_world
            el_world = self._el_world

            if self._gimbal_controller_mode == 'jacobian':
                # Jacobian J^{-1} controller (matching iris_ma6 training).
                # Uses quaternion error → proportional rate → J^{-1} → joint rates.
                # Includes body-rate feedforward for disturbance rejection.
                # Position targets: actual + qdot * dt (Jacobian-integrated).
                # Velocity targets: qdot (Jacobian).
                self._run_jacobian_ik(
                    az_world, el_world, actual_yaw, actual_roll, actual_pitch, dt)
            elif self._gimbal_controller_mode == 'combined':
                # Analytical position + Jacobian velocity (best of both worlds).
                # Position: exact atan2 IK (no tracking dynamics, no drift).
                # Velocity: J^{-1} feedforward (body-rate rejection + pointing rate).
                # The actuator PD tracks the exact position while the velocity
                # feedforward improves transient response and disturbance rejection.
                self._run_combined_ik(
                    az_world, el_world, actual_yaw, actual_roll, actual_pitch, dt)
            else:
                # Analytical IK: world az/el → body yaw/pitch + stabilizing roll.
                # Algebraically stable at any gimbal yaw angle — no dynamics.
                yaw_new, pitch_new = self._world_to_body_angles(
                    az_world, el_world, self._vehicle_quat_wxyz)
                roll_new = self._compute_stabilizing_roll(
                    yaw_new, self._vehicle_quat_wxyz)
                self._yaw = float(np.clip(yaw_new, self._yaw_limits[0], self._yaw_limits[1]))
                self._pitch = float(np.clip(pitch_new, self._pitch_limits[0], self._pitch_limits[1]))
                self._roll = float(np.clip(roll_new, self._roll_limits[0], self._roll_limits[1]))

        # Publish
        self._publish_joint_commands(dt)
        self._publish_state(az_world, el_world, actual_yaw, actual_roll,
                            actual_pitch, dt)

    def _run_jacobian_ik(
        self, az_world: float, el_world: float,
        actual_yaw: float, actual_roll: float, actual_pitch: float,
        dt: float,
    ):
        """Jacobian J^{-1} controller (matching iris_ma6 training).

        Algorithm:
          1. Compute desired camera quaternion from world-frame az/el
          2. Quaternion error: q_err = q_current * q_desired^{-1}
             (axis in body/current frame, NOT desired frame)
          3. omega_cmd = -K * att_error, rate-limited by servo_rate_limit
          4. qdot = J^{-1} * (omega_cmd - omega_body)
          5. target = actual + qdot * dt
        """
        # Desired camera quaternion in body frame
        q_desired = self._compute_desired_camera_quat(
            az_world, el_world, self._vehicle_quat_wxyz)

        # Current gimbal quaternion from actual joints
        q_current = self._gimbal_joints_to_quat(
            actual_yaw, actual_roll, actual_pitch)

        # Quaternion error — axis in current/body frame (fixed bug from iris_ma6)
        # q_err = q_current * q_desired^{-1}  (NOT q_desired^{-1} * q_current)
        q_err = _quat_mul(q_current, _quat_inv(q_desired))
        sign_w = 1.0 if q_err[0] >= 0 else -1.0
        att_error = 2.0 * sign_w * q_err[1:4]  # body-frame angular error

        # Pointing rate command with servo rate limiting
        omega_cmd = -self._pointing_gain * att_error
        omega_cmd = np.clip(omega_cmd, -self._servo_rate_limit, self._servo_rate_limit)

        # J^{-1} * (omega_cmd - omega_body)
        omega_combined = omega_cmd - self._body_angular_velocity_b
        self._qdot = self._jacobian_inv_times_omega(
            omega_combined, actual_yaw, actual_roll)

        # Position target = actual + joint rates * dt
        self._yaw = float(np.clip(
            actual_yaw + self._qdot[0] * dt,
            self._yaw_limits[0], self._yaw_limits[1]))
        self._roll = float(np.clip(
            actual_roll + self._qdot[1] * dt,
            self._roll_limits[0], self._roll_limits[1]))
        self._pitch = float(np.clip(
            actual_pitch + self._qdot[2] * dt,
            self._pitch_limits[0], self._pitch_limits[1]))

    def _run_combined_ik(
        self, az_world: float, el_world: float,
        actual_yaw: float, actual_roll: float, actual_pitch: float,
        dt: float,
    ):
        """Combined analytical position + Jacobian velocity targets.

        Position: exact atan2 IK — no tracking dynamics, no drift.
        Velocity: J^{-1}(omega_cmd - omega_body) — body-rate feedforward
                  for fast disturbance rejection during maneuvers.

        The actuator PD tracks the exact position target while the velocity
        feedforward reduces phase lag on transients.
        """
        # Position targets from analytical IK (exact, same as 'analytical' mode)
        yaw_new, pitch_new = self._world_to_body_angles(
            az_world, el_world, self._vehicle_quat_wxyz)
        roll_new = self._compute_stabilizing_roll(
            yaw_new, self._vehicle_quat_wxyz)

        self._yaw = float(np.clip(yaw_new, self._yaw_limits[0], self._yaw_limits[1]))
        self._pitch = float(np.clip(pitch_new, self._pitch_limits[0], self._pitch_limits[1]))
        self._roll = float(np.clip(roll_new, self._roll_limits[0], self._roll_limits[1]))

        # Velocity targets from Jacobian (feedforward)
        q_desired = self._compute_desired_camera_quat(
            az_world, el_world, self._vehicle_quat_wxyz)
        q_current = self._gimbal_joints_to_quat(
            actual_yaw, actual_roll, actual_pitch)

        q_err = _quat_mul(q_current, _quat_inv(q_desired))
        sign_w = 1.0 if q_err[0] >= 0 else -1.0
        att_error = 2.0 * sign_w * q_err[1:4]

        omega_cmd = -self._pointing_gain * att_error
        # omega_cmd = np.clip(omega_cmd, -self._servo_rate_limit, self._servo_rate_limit)

        omega_combined = omega_cmd - self._body_angular_velocity_b
        self._qdot = self._jacobian_inv_times_omega(
            omega_combined, actual_yaw, actual_roll)

    # ------------------------------------------------------------------
    # Jacobian-inverse math (ported from iris_ma6)
    # ------------------------------------------------------------------

    @staticmethod
    def _compute_desired_camera_quat(
        az: float, el: float, q_body: np.ndarray,
    ) -> np.ndarray:
        """Desired camera orientation as body-frame quaternion (wxyz).

        Camera frame: +X=forward (along LOS), +Z=up (horizon-stabilized).
        """
        fwd_world = np.array([
            math.cos(el) * math.cos(az),
            math.cos(el) * math.sin(az),
            math.sin(el)])
        world_up = np.array([0.0, 0.0, 1.0])

        fwd_body = _quat_rotate_inverse(q_body, fwd_world)
        up_body = _quat_rotate_inverse(q_body, world_up)

        # Gram-Schmidt orthonormalization
        cam_up = up_body - np.dot(up_body, fwd_body) * fwd_body
        cam_up = cam_up / (np.linalg.norm(cam_up) + 1e-8)
        cam_right = np.cross(cam_up, fwd_body)

        R = np.column_stack([fwd_body, cam_right, cam_up])
        return _rotmat_to_quat(R)

    @staticmethod
    def _gimbal_joints_to_quat(
        yaw: float, roll: float, pitch: float,
    ) -> np.ndarray:
        """Joint angles → quaternion wxyz. Order: Yaw(Z) → Roll(X) → Pitch(Y)."""
        q_yaw = _quat_from_euler_single(0, 0, yaw)
        q_roll = _quat_from_euler_single(roll, 0, 0)
        q_pitch = _quat_from_euler_single(0, pitch, 0)
        return _quat_mul(_quat_mul(q_yaw, q_roll), q_pitch)

    @staticmethod
    def _jacobian_inv_times_omega(
        omega: np.ndarray, gimbal_yaw: float, gimbal_roll: float,
    ) -> np.ndarray:
        """J^{-1} * omega for Yaw(Z)->Roll(X)->Pitch(Y) gimbal.

        Returns [yaw_dot, roll_dot, pitch_dot].
        """
        cy, sy = math.cos(gimbal_yaw), math.sin(gimbal_yaw)
        cr, sr = math.cos(gimbal_roll), math.sin(gimbal_roll)
        inv_cr = 1.0 / (cr + 1e-6 * (1.0 if cr >= 0 else -1.0))

        wx, wy, wz = omega
        yaw_dot = sy * sr * inv_cr * wx - cy * sr * inv_cr * wy + wz
        roll_dot = cy * wx + sy * wy
        pitch_dot = -sy * inv_cr * wx + cy * inv_cr * wy
        return np.array([yaw_dot, roll_dot, pitch_dot])

    # ------------------------------------------------------------------
    # Analytical IK (for position mode)
    # ------------------------------------------------------------------

    @staticmethod
    def _body_to_world_angles(
        yaw_body: float, pitch_body: float, q_wxyz: np.ndarray,
    ) -> tuple[float, float]:
        """Body-frame yaw/pitch → world-frame azimuth/elevation."""
        cos_p, sin_p = math.cos(pitch_body), math.sin(pitch_body)
        cos_y, sin_y = math.cos(yaw_body), math.sin(yaw_body)
        dir_body = np.array([cos_p * cos_y, cos_p * sin_y, -sin_p])
        dir_world = _quat_rotate(q_wxyz, dir_body)
        az = math.atan2(dir_world[1], dir_world[0])
        xy = math.sqrt(dir_world[0] ** 2 + dir_world[1] ** 2)
        el = math.atan2(dir_world[2], xy)
        return az, el

    @staticmethod
    def _world_to_body_angles(
        azimuth: float, elevation: float, q_wxyz: np.ndarray,
    ) -> tuple[float, float]:
        """World-frame azimuth/elevation → body-frame yaw/pitch."""
        dir_world = np.array([
            math.cos(elevation) * math.cos(azimuth),
            math.cos(elevation) * math.sin(azimuth),
            math.sin(elevation)])
        dir_body = _quat_rotate_inverse(q_wxyz, dir_world)
        yaw = math.atan2(dir_body[1], dir_body[0])
        xy = math.sqrt(dir_body[0] ** 2 + dir_body[1] ** 2)
        pitch = -math.atan2(dir_body[2], xy)
        return yaw, pitch

    @staticmethod
    def _compute_stabilizing_roll(
        gimbal_yaw: float, q_wxyz: np.ndarray,
    ) -> float:
        """Compute roll to keep horizon level (for position mode)."""
        world_up = np.array([0.0, 0.0, 1.0])
        up_in_body = _quat_rotate_inverse(q_wxyz, world_up)
        cos_yaw, sin_yaw = math.cos(gimbal_yaw), math.sin(gimbal_yaw)
        up_y_yawed = -up_in_body[0] * sin_yaw + up_in_body[1] * cos_yaw
        up_z_yawed = up_in_body[2]
        return math.atan2(-up_y_yawed, up_z_yawed)

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------

    def _publish_joint_commands(self, dt: float):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self._joint_names

        positions = [
            self._yaw + YAW_JOINT_OFFSET,
            -self._roll,   # negate: internal → USD
            -self._pitch,  # negate: internal → USD
        ]
        if self._qdot is not None:
            # Jacobian-computed velocities (combined/jacobian mode)
            velocities = [
                float(self._qdot[0]),
                float(-self._qdot[1]),  # negate: internal → USD
                float(-self._qdot[2]),  # negate: internal → USD
            ]
            self._qdot = None  # consumed
        else:
            # Finite-difference fallback (analytical mode)
            velocities = [
                (self._yaw - self._prev_actual_yaw) / dt,
                -(self._roll - self._prev_actual_roll) / dt,
                -(self._pitch - self._prev_actual_pitch) / dt,
            ]

        if self._control_mode == 'position':
            msg.position = positions
        elif self._control_mode == 'position_velocity':
            msg.position = positions
            msg.velocity = velocities
        elif self._control_mode == 'velocity':
            msg.velocity = velocities

        self._cmd_pub.publish(msg)

    def _publish_state(self, az_world: float, el_world: float,
                       actual_yaw: float, actual_roll: float,
                       actual_pitch: float, dt: float):
        # Body-frame RPY in degrees for downstream Rz(yaw)*Rx(roll)*Ry(pitch).
        # Roll: internal positive=CW(front), Rx positive=CCW(front) → publish as-is
        #   (empirically verified: frustum roll matches camera image)
        # If -actual_pitch: Pitch: internal positive=down, Ry positive=up → negate
        # If actual_pitch: Pitch: internal positive=down, Ry positive=down → as-is
        rpy_deg = Vector3()
        rpy_deg.x = math.degrees(actual_roll)
        rpy_deg.y = math.degrees(actual_pitch)
        rpy_deg.z = math.degrees(actual_yaw)
        self._rpy_deg_pub.publish(rpy_deg)

        # World-frame LOS in degrees
        los = Vector3()
        los.x = math.degrees(az_world)
        los.y = math.degrees(el_world)
        los.z = 0.0
        self._los_state_pub.publish(los)

        # Combined angular velocity (body + gimbal) in world frame.
        # Uses full ZXY Jacobian matching training env
        # (derived_field_computers.py:compute_combined_angular_velocity).
        # Gimbal chain: Yaw(Z) -> Roll(X) -> Pitch(Y)
        # omega_gimbal_b = yaw_rate*[0,0,1]
        #   + roll_rate * Rz(yaw) @ [1,0,0]
        #   + pitch_rate * Rz(yaw) @ Rx(roll) @ [0,1,0]
        gimbal_yaw_rate = (actual_yaw - self._prev_actual_yaw) / dt
        gimbal_roll_rate = (actual_roll - self._prev_actual_roll) / dt
        gimbal_pitch_rate = (actual_pitch - self._prev_actual_pitch) / dt
        cy, sy = math.cos(actual_yaw), math.sin(actual_yaw)
        cr, sr = math.cos(actual_roll), math.sin(actual_roll)
        gimbal_ang_vel_b = np.array([
            -sy * cr * gimbal_pitch_rate + cy * gimbal_roll_rate,   # X
             cy * cr * gimbal_pitch_rate + sy * gimbal_roll_rate,   # Y
             sr * gimbal_pitch_rate + gimbal_yaw_rate,              # Z
        ])
        combined_b = self._body_angular_velocity_b + gimbal_ang_vel_b
        combined_w = _quat_rotate(self._vehicle_quat_wxyz, combined_b)
        cav_msg = Vector3Stamped()
        cav_msg.header.stamp = self.get_clock().now().to_msg()
        cav_msg.vector.x = float(combined_w[0])
        cav_msg.vector.y = float(combined_w[1])
        cav_msg.vector.z = float(combined_w[2])
        self._combined_ang_vel_w_pub.publish(cav_msg)

        # Update previous actual positions
        self._prev_actual_yaw = actual_yaw
        self._prev_actual_roll = actual_roll
        self._prev_actual_pitch = actual_pitch

        # Zoom: integrate rate command (already in zoom-levels/s) and publish
        if self._zoom_cmd_rate != 0.0:
            self._zoom_level += self._zoom_cmd_rate * dt
            self._zoom_level = max(self._zoom_min, min(self._zoom_max, self._zoom_level))
            cam_zoom_cmd_msg = Float64()
            cam_zoom_cmd_msg.data = self._zoom_level
            self._camera_zoom_cmd_pub.publish(cam_zoom_cmd_msg)

        zoom_level_msg = Float64()
        zoom_level_msg.data = self._zoom_level
        self._zoom_level_pub.publish(zoom_level_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LOSRateController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
