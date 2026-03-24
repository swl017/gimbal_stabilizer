#!/usr/bin/env python3
from __future__ import annotations
"""World-frame LOS-rate gimbal controller for RL policy deployment.

Ports the iris_ma6 analytical gimbal controller to ROS2. Accepts normalized
world-frame azimuth/elevation rate commands, integrates them, and computes
body-frame joint positions via inverse kinematics.

Algorithm (from gimbal_controller_analytical.py):
  1. Feedback blend: correct internal state drift from actual joints
  2. World-frame rate integration: az += cmd * max_rate * dt
  3. World-to-body IK: atan2 decomposition of target direction in body frame
  4. Stabilizing roll: project world-up into yawed gimbal frame
  5. Clamp to joint limits
  6. Publish joint position targets

Frame Conventions:
- WORLD (ENU): +X=East, +Y=North, +Z=Up
- BODY (FLU):  +X=Forward, +Y=Left, +Z=Up
- Gimbal joint order: Yaw(Z) -> Roll(X) -> Pitch(Y)
- YAW_JOINT_OFFSET = -pi/2 (body mesh visual rotation compensation)

Quaternion Convention:
- ROS2 Imu msg: xyzw
- Internal math: wxyz (matching Isaac Lab / iris_ma6)
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Header

# Body mesh is rotated 90 deg CCW from physics frame (visual forward = body +Y).
# This offset is added to yaw joint targets so that controller yaw=0 maps to
# body +X (physics forward).
YAW_JOINT_OFFSET = -math.pi / 2

# Joint name mapping per gimbal model.  Order: [yaw, roll, pitch].
# Cross-referenced against USD assets in PegasusSimulator/.../assets/Robots/.
GIMBAL_MODELS: dict[str, list[str]] = {
    'cgo3': [
        'cgo3_vertical_arm_joint',     # yaw  — iris_gimbal.usda, typhoon_h480.usda
        'cgo3_horizontal_arm_joint',   # roll
        'cgo3_camera_joint',           # pitch
    ],
    'iris_gimbal3': [
        'yaw_joint',                   # yaw  — iris_gimbal3.usda, iris_gimbal2.usda
        'roll_joint',                  # roll
        'pitch_joint',                 # pitch
    ],
}


def _quat_rotate_inverse(q_wxyz: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate vector v by the inverse of quaternion q (wxyz format).

    Uses the cross-product form: v' = v + 2w*(u x v) + 2*(u x (u x v))
    where u = -[x,y,z] (conjugate).
    """
    w = q_wxyz[0]
    u = -q_wxyz[1:4]  # conjugate: negate xyz
    t = 2.0 * np.cross(u, v)
    return v + w * t + np.cross(u, t)


class LOSRateController(Node):
    """World-frame LOS-rate gimbal controller.

    Subscribes to normalized azimuth/elevation rate commands and vehicle IMU,
    computes body-frame joint positions via analytical inverse kinematics,
    and publishes joint position targets.
    """

    def __init__(self):
        super().__init__('los_rate_controller')

        # -- Declare parameters with iris_ma6 defaults --
        self.declare_parameter('max_gimbal_rate', 2.0 * math.pi)
        self.declare_parameter('yaw_limits_deg', [-160.0, 160.0])
        self.declare_parameter('pitch_limits_deg', [-45.0, 45.0])
        self.declare_parameter('roll_limits_deg', [-45.0, 45.0])
        self.declare_parameter('feedback_blend', 0.05)
        self.declare_parameter('update_rate', 100.0)
        self.declare_parameter('model', 'cgo3')
        self.declare_parameter('control_mode', 'position')

        self._max_gimbal_rate = self.get_parameter('max_gimbal_rate').value
        yaw_lim_deg = self.get_parameter('yaw_limits_deg').value
        pitch_lim_deg = self.get_parameter('pitch_limits_deg').value
        roll_lim_deg = self.get_parameter('roll_limits_deg').value
        self._yaw_limits = (math.radians(yaw_lim_deg[0]), math.radians(yaw_lim_deg[1]))
        self._pitch_limits = (math.radians(pitch_lim_deg[0]), math.radians(pitch_lim_deg[1]))
        self._roll_limits = (math.radians(roll_lim_deg[0]), math.radians(roll_lim_deg[1]))
        self._feedback_blend = self.get_parameter('feedback_blend').value
        self._update_rate = self.get_parameter('update_rate').value
        self._control_mode = self.get_parameter('control_mode').value

        # -- Resolve joint names from model --
        model = self.get_parameter('model').value
        if model not in GIMBAL_MODELS:
            raise ValueError(
                f"Unknown gimbal model '{model}'. "
                f"Supported: {list(GIMBAL_MODELS.keys())}")

        # -- Internal state --
        # World-frame target angles (rate-integrated pointing direction)
        self._azimuth_world = 0.0
        self._elevation_world = 0.0
        # Body-frame joint angle targets
        self._yaw = 0.0
        self._roll = 0.0
        self._pitch = 0.0
        # Previous-step targets for finite-difference velocity
        self._yaw_prev = 0.0
        self._roll_prev = 0.0
        self._pitch_prev = 0.0

        # -- Cached subscriber data --
        self._vehicle_quat_wxyz: np.ndarray | None = None
        self._joint_positions_actual: dict | None = None
        self._cmd_az_rate = 0.0
        self._cmd_el_rate = 0.0

        # -- Joint names (resolved from model) --
        self._joint_names = GIMBAL_MODELS[model]

        # -- QoS profiles --
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # -- Publishers --
        self._cmd_pub = self.create_publisher(
            JointState, 'isaac_joint_commands', reliable_qos)
        self._rpy_rad_pub = self.create_publisher(
            Vector3, 'gimbal_state_rpy_rad', sensor_qos)
        self._rpy_deg_pub = self.create_publisher(
            Vector3, 'gimbal_state_rpy_deg', sensor_qos)
        self._los_state_pub = self.create_publisher(
            Vector3, 'gimbal_los_state', sensor_qos)

        # -- Subscribers --
        self.create_subscription(
            Vector3, 'gimbal_cmd_los_rate',
            self._los_rate_cmd_callback, sensor_qos)
        self.create_subscription(
            Imu, 'mavros/imu/data',
            self._imu_callback, sensor_qos)
        self.create_subscription(
            JointState, 'isaac_joint_states',
            self._joint_state_callback, sensor_qos)

        # -- Timer --
        timer_period = 1.0 / self._update_rate
        self.create_timer(timer_period, self._timer_callback)

        self.get_logger().info(
            f'LOS rate controller started: model={model}, '
            f'rate={self._update_rate} Hz, '
            f'max_gimbal_rate={math.degrees(self._max_gimbal_rate):.1f} deg/s, '
            f'feedback_blend={self._feedback_blend}, '
            f'control_mode={self._control_mode}')

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------

    def _los_rate_cmd_callback(self, msg: Vector3):
        """Cache normalized LOS rate commands. x=azimuth, y=elevation, [-1,1]."""
        self._cmd_az_rate = float(np.clip(msg.x, -1.0, 1.0))
        self._cmd_el_rate = float(np.clip(msg.y, -1.0, 1.0))

    def _imu_callback(self, msg: Imu):
        """Cache vehicle quaternion, converting ROS2 xyzw to wxyz."""
        q = msg.orientation
        self._vehicle_quat_wxyz = np.array([q.w, q.x, q.y, q.z])

    def _joint_state_callback(self, msg: JointState):
        """Parse and cache actual joint positions by name."""
        positions = {}
        yaw_name, roll_name, pitch_name = self._joint_names
        for i, name in enumerate(msg.name):
            if name == yaw_name:
                positions['yaw'] = msg.position[i]
            elif name == roll_name:
                positions['roll'] = msg.position[i]
            elif name == pitch_name:
                positions['pitch'] = msg.position[i]
        if len(positions) == 3:
            self._joint_positions_actual = positions

    # ------------------------------------------------------------------
    # Timer callback — main control loop
    # ------------------------------------------------------------------

    def _timer_callback(self):
        """Compute and publish gimbal joint targets at update_rate Hz."""
        if self._vehicle_quat_wxyz is None:
            return

        dt = 1.0 / self._update_rate

        # 1. Feedback blend: correct internal state drift from actual joints
        if self._joint_positions_actual is not None and self._feedback_blend > 0.0:
            actual_yaw = self._joint_positions_actual['yaw'] - YAW_JOINT_OFFSET
            actual_roll = self._joint_positions_actual['roll']
            actual_pitch = self._joint_positions_actual['pitch']
            beta = self._feedback_blend
            self._yaw += beta * (actual_yaw - self._yaw)
            self._roll += beta * (actual_roll - self._roll)
            self._pitch += beta * (actual_pitch - self._pitch)

        # 2. World-frame rate integration
        az_rate = self._cmd_az_rate * self._max_gimbal_rate
        el_rate = self._cmd_el_rate * self._max_gimbal_rate
        self._azimuth_world += az_rate * dt
        self._elevation_world += el_rate * dt
        # Wrap azimuth to [-pi, pi]
        self._azimuth_world = math.atan2(
            math.sin(self._azimuth_world), math.cos(self._azimuth_world))
        # Clamp elevation to pitch limits
        self._elevation_world = float(np.clip(
            self._elevation_world, self._pitch_limits[0], self._pitch_limits[1]))

        # 3. World-to-body angle conversion (LOS: 2-DOF)
        yaw_new, pitch_new = self._world_to_body_angles(
            self._azimuth_world, self._elevation_world, self._vehicle_quat_wxyz)

        # 4. Stabilizing roll (horizon: 1-DOF)
        roll_new = self._compute_stabilizing_roll(yaw_new, self._vehicle_quat_wxyz)

        # 5. Update state (save previous for finite-difference velocity)
        self._yaw_prev = self._yaw
        self._roll_prev = self._roll
        self._pitch_prev = self._pitch
        self._yaw = yaw_new
        self._pitch = pitch_new
        self._roll = roll_new

        # 6. Clamp to joint limits
        self._yaw = float(np.clip(self._yaw, self._yaw_limits[0], self._yaw_limits[1]))
        self._pitch = float(np.clip(self._pitch, self._pitch_limits[0], self._pitch_limits[1]))
        self._roll = float(np.clip(self._roll, self._roll_limits[0], self._roll_limits[1]))

        # 7. Publish
        self._publish_joint_commands()
        self._publish_state()

    # ------------------------------------------------------------------
    # Analytical inverse kinematics (ported from iris_ma6)
    # ------------------------------------------------------------------

    @staticmethod
    def _world_to_body_angles(
        azimuth: float, elevation: float, q_wxyz: np.ndarray,
    ) -> tuple[float, float]:
        """Convert world-frame pointing to body-frame yaw/pitch.

        Projects world-frame target direction into body frame via q_body
        inverse rotation, then extracts yaw and pitch via atan2.
        """
        cos_az = math.cos(azimuth)
        sin_az = math.sin(azimuth)
        cos_el = math.cos(elevation)
        sin_el = math.sin(elevation)

        dir_world = np.array([cos_el * cos_az, cos_el * sin_az, sin_el])
        dir_body = _quat_rotate_inverse(q_wxyz, dir_world)

        yaw_body = math.atan2(dir_body[1], dir_body[0])
        xy_dist = math.sqrt(dir_body[0] ** 2 + dir_body[1] ** 2)
        pitch_body = -math.atan2(dir_body[2], xy_dist)

        return yaw_body, pitch_body

    @staticmethod
    def _compute_stabilizing_roll(
        gimbal_yaw: float, q_wxyz: np.ndarray,
    ) -> float:
        """Compute roll to keep horizon level.

        Projects world-up into the yawed gimbal frame and computes the roll
        to align camera up-axis with projected world-up.
        """
        world_up = np.array([0.0, 0.0, 1.0])
        up_in_body = _quat_rotate_inverse(q_wxyz, world_up)

        cos_yaw = math.cos(gimbal_yaw)
        sin_yaw = math.sin(gimbal_yaw)

        up_y_yawed = -up_in_body[0] * sin_yaw + up_in_body[1] * cos_yaw
        up_z_yawed = up_in_body[2]

        return math.atan2(-up_y_yawed, up_z_yawed)

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------

    def _publish_joint_commands(self):
        """Publish joint commands with YAW_JOINT_OFFSET applied.

        Control mode selects which JointState fields are populated:
          position           — .position only (default)
          position_velocity  — .position + .velocity (PD with feedforward)
          velocity           — .velocity only (pure rate control)
        """
        dt = 1.0 / self._update_rate
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self._joint_names

        positions = [
            self._yaw + YAW_JOINT_OFFSET,
            self._roll,
            self._pitch,
        ]
        velocities = [
            (self._yaw - self._yaw_prev) / dt,
            (self._roll - self._roll_prev) / dt,
            (self._pitch - self._pitch_prev) / dt,
        ]

        if self._control_mode == 'position':
            msg.position = positions
        elif self._control_mode == 'position_velocity':
            msg.position = positions
            msg.velocity = velocities
        elif self._control_mode == 'velocity':
            msg.velocity = velocities

        self._cmd_pub.publish(msg)

    def _publish_state(self):
        """Publish gimbal state for monitoring."""
        # Body-frame RPY in radians
        rpy_rad = Vector3()
        rpy_rad.x = self._roll
        rpy_rad.y = self._pitch
        rpy_rad.z = self._yaw
        self._rpy_rad_pub.publish(rpy_rad)

        # Body-frame RPY in degrees
        rpy_deg = Vector3()
        rpy_deg.x = math.degrees(self._roll)
        rpy_deg.y = math.degrees(self._pitch)
        rpy_deg.z = math.degrees(self._yaw)
        self._rpy_deg_pub.publish(rpy_deg)

        # World-frame azimuth/elevation
        los = Vector3()
        los.x = self._azimuth_world
        los.y = self._elevation_world
        los.z = 0.0
        self._los_state_pub.publish(los)


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
