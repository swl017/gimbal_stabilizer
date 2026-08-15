#!/usr/bin/env python3
from __future__ import annotations
"""Fixed camera mount — the sim stand-in for a rigidly bolted, non-gimballed camera.

MAS ticket 055.  The real interceptor has NO gimbal and NO zoom hardware: the camera
is bolted forward and pitched 30.8 deg UP in body-FLU (calibration-confirmed).  The
field stack therefore does not run a gimbal controller at all — it shims two constants
onto the wire (`mas_fieldtest/src/tmux/intercept_mas.yaml:119-120`):

    ros2 topic pub -r 20 /gimbal_state_rpy_deg geometry_msgs/msg/Vector3 "{x: 0.0, y: -30.8, z: 0.0}"
    ros2 topic pub -r 5  /camera/zoom_level    std_msgs/msg/Float64      "{data: 1.0}"

This node is those two shims plus the one thing the sim additionally needs: a constant
`isaac_joint_commands` holding the Isaac gimbal at the equivalent pose.  It REPLACES
`los_rate_controller` in the fixed-camera session; the two must never run together on
the same namespace (both publish `isaac_joint_commands`).

Why constants and not a controller: matching the field is the entire point.  A rigid
mount has no tracking dynamics, no rate loop, no body-motion rejection.  Anything this
node computed per-tick would be a sim artifact with no counterpart in the aircraft.

── Sign chain (ticket 055 r_research.md §5.1; verified across three files) ──────────
The USD joint signs are inverted with respect to the controller's internal convention,
and the yaw joint carries a mesh offset.  Both are defined by `los_rate_controller`,
and are imported from it here rather than restated — a duplicated convention constant
is exactly the thing that drifts.

    internal yaw   0            -> USD yaw_joint   = +pi/2   (YAW_JOINT_OFFSET)
                                   (at yaw_joint = 0 the camera looks along body -Y,
                                    i.e. out the RIGHT side — commanding pitch alone
                                    is the classic way to get this wrong)
    internal roll  = -USD roll_joint
    internal pitch = -USD pitch_joint,  internal pitch POSITIVE = DOWN

    gimbal_state_rpy_deg = (internal roll, internal pitch, internal yaw) in degrees
                           (`los_rate_controller._publish_state`)

So a mount pitched UP by 30.8 deg is:

    internal pitch  = -30.8 deg   ->  gimbal_state_rpy_deg.y = -30.8   (== the field shim)
    USD pitch_joint = +30.8 deg   =  +0.5376 rad

── What this node deliberately does NOT publish ────────────────────────────────────
`camera/zoom_level_cmd`.  Pegasus's ROS2Backend subscribes it and calls
`MonocularCamera.set_zoom`, which rebuilds the camera's focal length as MEAN(fx, fy)
from the configured intrinsics.  That is a harmless no-op on a square-pixel camera and
a silent 444.76 -> 388.87 px miscalibration on the interceptor's anisotropic one
(ticket 055 F2).  The sim's camera is fixed-focal; nothing should be telling it to zoom.
`camera/zoom_level` (the STATE topic the mas consumers read) is still published, at 1.0.

── Deflection monitor (ticket 055 AC4) ─────────────────────────────────────────────
Commanding a constant joint angle is not the same as a rigid mount: the Isaac joint is
a PD drive (stiffness 1000, damping 50 on ~1 g links) and will deflect transiently under
body angular acceleration.  `gimbal_state_rpy_deg` still carries the CONSTANT — that is
what the field publishes, and it is what makes sim and field agree by construction — so
the deflection is reported out of band, on `gimbal_mount_deflection_deg` and in a
throttled warning, where it can be measured without contaminating the estimator input.
"""

import math
import time

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Header

# Single source of truth for the joint names and the yaw mesh offset.  Importing them
# keeps this node and the controller it replaces from disagreeing about the airframe.
from .los_rate_controller import GIMBAL_MODELS, YAW_JOINT_OFFSET


class FixedMountPublisher(Node):
    """Holds the gimbal at a constant body-frame pose and shims the field's constants."""

    def __init__(self):
        super().__init__('fixed_mount_publisher')

        # -- Mount pose.  Defaults are the flown interceptor: forward, 30.8 deg UP. --
        # `mount_pitch_up_deg` is POSITIVE-UP, matching how the mount is described in
        # the field config, NOT the internal positive-down convention.  The negation
        # happens once, below, where it is visible next to the field shim it reproduces.
        self.declare_parameter('mount_pitch_up_deg', 30.8)
        self.declare_parameter('mount_roll_deg', 0.0)
        self.declare_parameter('mount_yaw_deg', 0.0)   # +left, about body +Z, 0 = forward

        self.declare_parameter('model', 'iris_gimbal3')
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('zoom_level', 1.0)
        self.declare_parameter('publish_zoom_level', True)

        # Ticket 055 AC4 instrumentation.  Off-the-wire; see the module docstring.
        self.declare_parameter('monitor_deflection', True)
        self.declare_parameter('deflection_warn_deg', 0.5)

        # Ticket 055 D2 — the Isaac/ROS consistency guard.  See _check_boot_label.
        self.declare_parameter('check_boot_label', True)
        self.declare_parameter('boot_label_path', '/tmp/isaac_boot_label.json')
        # How long to WAIT for the label before giving up on it.  Non-obvious and
        # load-bearing: measured at bring-up (2026-08-15), the ROS side starts ~5 s after
        # tmuxp load while Isaac needs ~60 s to reach `world.reset()` and write the label.
        # A single check at startup therefore ALWAYS missed it and the guard silently
        # degraded to its warn-and-proceed path — i.e. it never once did its job.
        self.declare_parameter('boot_label_wait_s', 180.0)

        model = self.get_parameter('model').value
        if model not in GIMBAL_MODELS:
            raise ValueError(
                f"Unknown gimbal model '{model}'. "
                f"Supported: {list(GIMBAL_MODELS.keys())}")
        self._joint_names = GIMBAL_MODELS[model]      # [yaw, roll, pitch]

        pitch_up_deg = float(self.get_parameter('mount_pitch_up_deg').value)
        roll_deg = float(self.get_parameter('mount_roll_deg').value)
        yaw_deg = float(self.get_parameter('mount_yaw_deg').value)

        # Internal convention: pitch POSITIVE = DOWN, so a mount pitched UP is negative.
        self._internal_roll = math.radians(roll_deg)
        self._internal_pitch = -math.radians(pitch_up_deg)
        self._internal_yaw = math.radians(yaw_deg)

        # USD joint targets: negate roll/pitch, add the yaw mesh offset.
        self._usd_positions = [
            self._internal_yaw + YAW_JOINT_OFFSET,
            -self._internal_roll,
            -self._internal_pitch,
        ]

        rate_hz = float(self.get_parameter('publish_rate_hz').value)
        if rate_hz <= 0.0:
            raise ValueError(f'publish_rate_hz must be > 0 (got {rate_hz})')

        # Deferred, not called inline: the label does not exist yet at construction time (see
        # `boot_label_wait_s`). Publishing the mount pose meanwhile is correct — if the check
        # later fails the node dies, and until then a fixed mount is the right thing to hold.
        self._boot_label_deadline = None
        if bool(self.get_parameter('check_boot_label').value):
            self._boot_label_deadline = (
                time.monotonic() + float(self.get_parameter('boot_label_wait_s').value))
            self._boot_label_timer = self.create_timer(2.0, self._poll_boot_label)

        self._zoom_level = float(self.get_parameter('zoom_level').value)
        self._publish_zoom = bool(self.get_parameter('publish_zoom_level').value)
        self._monitor = bool(self.get_parameter('monitor_deflection').value)
        self._warn_rad = math.radians(
            float(self.get_parameter('deflection_warn_deg').value))

        # -- QoS: identical to los_rate_controller's, so swapping the two nodes does
        #    not silently change delivery semantics for any existing subscriber. --
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=10)

        self._cmd_pub = self.create_publisher(
            JointState, 'isaac_joint_commands', reliable_qos)
        self._rpy_deg_pub = self.create_publisher(
            Vector3, 'gimbal_state_rpy_deg', sensor_qos)
        self._zoom_level_pub = self.create_publisher(
            Float64, 'camera/zoom_level', sensor_qos) if self._publish_zoom else None
        # NOTE: camera/zoom_level_cmd is deliberately absent — see the module docstring.

        # -- Deflection monitor --
        self._peak_deflection = [0.0, 0.0, 0.0]       # |actual - commanded|, USD frame
        self._armed = False                          # see _joint_state_callback
        self._deflection_pub = None
        if self._monitor:
            self._deflection_pub = self.create_publisher(
                Vector3, 'gimbal_mount_deflection_deg', sensor_qos)
            joint_qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST, depth=50)
            self.create_subscription(
                JointState, 'isaac_joint_states',
                self._joint_state_callback, joint_qos)

        self.create_timer(1.0 / rate_hz, self._timer_callback)

        # Ticket 055 D2, the other half of the consistency problem. The boot label catches
        # "Isaac says this vehicle is gimballed"; this catches "a los_rate_controller is
        # ALSO driving this vehicle's joints". Both would publish isaac_joint_commands and
        # the articulation would follow whichever arrived last, so the mount would judder
        # instead of holding — again looking like a tracking fault, not a config fault.
        # Deferred rather than checked in __init__: the two nodes usually start together.
        self._contention_timer = self.create_timer(5.0, self._check_command_contention)

        self.get_logger().info(
            f'Fixed camera mount: model={model}, '
            f'pitch={pitch_up_deg:+.2f} deg UP, roll={roll_deg:+.2f}, '
            f'yaw={yaw_deg:+.2f} deg -> USD [{self._joint_names[0]}, '
            f'{self._joint_names[1]}, {self._joint_names[2]}] = '
            f'[{self._usd_positions[0]:+.4f}, {self._usd_positions[1]:+.4f}, '
            f'{self._usd_positions[2]:+.4f}] rad; '
            f'gimbal_state_rpy_deg = ({roll_deg:.1f}, '
            f'{-pitch_up_deg:.1f}, {yaw_deg:.1f}); '
            f'zoom={"%.2f" % self._zoom_level if self._publish_zoom else "not published"}; '
            f'rate={rate_hz:.1f} Hz')

    # ------------------------------------------------------------------

    def _poll_boot_label(self):
        """Retry `_check_boot_label` until the label appears or the deadline passes.

        Isaac writes the label at `world.reset()`, roughly a minute after the tmuxp session
        starts the ROS nodes. Checking once at construction — which is what this node did
        until bring-up on 2026-08-15 — therefore always found nothing and fell through to the
        warn-and-proceed path, so the guard never actually guarded anything. The bug was
        invisible offline because every unit test wrote the label first.
        """
        found = self._check_boot_label()
        if found:
            self._boot_label_timer.cancel()
            return
        if time.monotonic() > self._boot_label_deadline:
            self._boot_label_timer.cancel()
            self.get_logger().warn(
                f'gave up waiting for a usable Isaac boot label after '
                f'{self.get_parameter("boot_label_wait_s").value:.0f} s. Running UNVERIFIED: '
                f'if Isaac has this vehicle in the gimballed set, this node is holding a '
                f'gimbal that still wears the pinhole lens (ticket 055 D2).')

    def _check_boot_label(self):
        """Refuse to start unless Isaac agrees this vehicle has a FIXED camera (055 D2).

        Returns True once a live, ticket-055-era label has been read and checked (so the
        caller can stop polling); False while the label is absent, stale, or pre-055.

        The failure this closes: the Isaac side picks the fixed-fisheye vehicles via
        `FIXED_CAMERA_VEHICLES`, the ROS side picks the fixed-mount vehicles via the launch
        file's `namespaces:=`. Two lists, two delivery mechanisms, and until now nothing
        checking they agree — while the launcher's own header documents exactly why they
        drift: `VAR=... tmuxp load` is silently dropped whenever a tmux server exists, and an
        idle `keepalive` session keeps one alive on this box permanently.

        A disagreement is silent and looks like a tracking bug rather than a config bug:
        fisheye intrinsics on a gimbal being swung by a LOS-rate controller, or a rigid mount
        still wearing the 1053 px pinhole. So the Isaac launcher writes its selection into the
        boot label and this node reads it back.

        STALENESS is the hazard a well-known path invites, so the label carries the writing
        process's pid and this check requires /proc/<pid> to be alive — a label left behind by
        a previous session is treated as no label at all. Same contract the launcher's own
        docstring sets out.

        Fail-closed on DISAGREEMENT (Isaac ran and says this vehicle is not fixed), but only
        warn when the label is absent or stale: this node is also usable against a hand-started
        sim, and refusing to run because a file is missing would be a worse failure than the
        one being prevented. Set `check_boot_label:=false` to skip entirely.
        """
        import json
        import os

        path = str(self.get_parameter('boot_label_path').value)
        ns = self.get_namespace().strip('/') or ''
        if not ns:
            self.get_logger().warn(
                'running in the root namespace — cannot check the boot label for a vehicle '
                'name (ticket 055 D2). Launch this node under a /px4_N namespace.')
            return True     # terminal: no amount of waiting supplies a namespace

        try:
            with open(path) as f:
                label = json.load(f)
        except (OSError, ValueError) as e:
            # Expected for the first ~minute of a session — Isaac writes the label at
            # world.reset(), long after the ROS nodes start. Throttled so the wait is
            # visible without burying the console.
            self.get_logger().info(
                f'waiting for the Isaac boot label at {path} ({type(e).__name__}) — holding '
                f'the mount pose meanwhile (ticket 055 D2)', throttle_duration_sec=20.0)
            return False

        pid = label.get('pid')
        if not (isinstance(pid, int) and os.path.exists(f'/proc/{pid}')):
            self.get_logger().warn(
                f'boot label at {path} is STALE (pid={pid} is not running) — it describes a '
                f'previous session. Ignoring it rather than trusting it; still waiting for a '
                f'live one (ticket 055 D2).', throttle_duration_sec=20.0)
            return False

        fixed = label.get('fixed_camera_vehicles')
        if fixed is None:
            self.get_logger().warn(
                f'boot label at {path} has no `fixed_camera_vehicles` — it was written by a '
                f'launcher predating ticket 055 (probably px4_multi_world_iris_gimbal3). '
                f'That launcher renders the 1053 px PINHOLE camera, so a fixed mount here is '
                f'geometrically meaningless. Proceeding anyway; check which launcher is up.')
            return True     # terminal: this Isaac is live and will not grow the field

        try:
            vehicle_id = int(ns.rsplit('_', 1)[-1])
        except ValueError:
            self.get_logger().warn(
                f"cannot parse a vehicle id out of namespace '{ns}' — skipping the boot "
                f"label check (ticket 055 D2).")
            return True     # terminal

        if vehicle_id not in fixed:
            raise RuntimeError(
                f'Isaac says vehicle {vehicle_id} is NOT a fixed-camera vehicle '
                f'(fixed_camera_vehicles={fixed}, camera_vehicles='
                f'{label.get("camera_vehicles")}), but a fixed_mount_publisher was launched '
                f'for {ns}. That combination gives a rigidly-held gimbal wearing the '
                f'gimballed pinhole lens, and it reads downstream as a tracking failure '
                f'rather than a configuration error. Fix FIXED_CAMERA_VEHICLES or the '
                f'launch `namespaces:=` so they agree — and remember `VAR=... tmuxp load` is '
                f'silently dropped when a tmux server exists; use the tmuxp `environment:` '
                f'block or `tmux setenv -g` and verify with `tmux show-environment -g`. '
                f'[ticket 055 D2]')

        self.get_logger().info(
            f'boot label OK: Isaac (pid {pid}) lists vehicle {vehicle_id} in '
            f'fixed_camera_vehicles={fixed}')
        return True

    def _check_command_contention(self):
        """Warn if something else is also commanding this vehicle's joints (055 D2).

        One-shot, once the graph has settled. `count_publishers` includes this node, so more
        than one means a second writer — in a mixed session that is a `los_rate_controller`
        launched over a namespace that should have been left to the fixed mount.
        """
        self._contention_timer.cancel()
        topic = self.resolve_topic_name('isaac_joint_commands')
        n = self.count_publishers(topic)
        if n > 1:
            self.get_logger().error(
                f'{n} publishers on {topic} — something else is commanding this gimbal, '
                f'almost certainly a los_rate_controller launched over a namespace that '
                f'should be fixed-mount only. The articulation follows whichever message '
                f'arrives last, so the mount judders instead of holding and it reads as a '
                f'tracking fault. Narrow the LOS-rate launch (config_file:= or namespaces:=) '
                f'so the two sets do not overlap [ticket 055 D2].')

    def _timer_callback(self):
        stamp = self.get_clock().now().to_msg()

        cmd = JointState()
        cmd.header = Header()
        cmd.header.stamp = stamp
        cmd.name = self._joint_names
        cmd.position = list(self._usd_positions)
        self._cmd_pub.publish(cmd)

        # The constant the field shims in. Byte-identical by construction.
        rpy = Vector3()
        rpy.x = math.degrees(self._internal_roll)
        rpy.y = math.degrees(self._internal_pitch)
        rpy.z = math.degrees(self._internal_yaw)
        self._rpy_deg_pub.publish(rpy)

        if self._zoom_level_pub is not None:
            zoom = Float64()
            zoom.data = self._zoom_level
            self._zoom_level_pub.publish(zoom)

        if self._deflection_pub is not None:
            defl = Vector3()
            defl.x = math.degrees(self._peak_deflection[1])   # roll
            defl.y = math.degrees(self._peak_deflection[2])   # pitch
            defl.z = math.degrees(self._peak_deflection[0])   # yaw
            self._deflection_pub.publish(defl)

    def _joint_state_callback(self, msg: JointState):
        """Track peak |achieved - commanded| per joint (ticket 055 AC4).

        Peak-hold, not instantaneous: the quantity of interest is the worst deflection over
        an engagement, and a transient at 250 Hz is easy to miss in an echo.

        ARMED ON FIRST CONVERGENCE, and that is the whole point. The joints start at zero
        while the mount pose is yaw +90 deg / pitch +30.8, so the initial slew registers as
        exactly that much "deflection". Bring-up on 2026-08-15 duly reported a peak of
        90.00 deg on yaw_joint and 30.80 on pitch — the commanded offsets, restated. That
        number is not a measurement of anything, and it would have sat there for the whole
        session masking the real figure (which turned out to be 0.002 deg once holding).
        So the peak-hold starts only after the mount has reached its pose once; before that
        the deflection topic reports zeros and says so in the log.
        """
        worst_name, worst_err = None, 0.0
        errs = [None] * 3
        for idx, name in enumerate(self._joint_names):
            try:
                pos = msg.position[msg.name.index(name)]
            except (ValueError, IndexError):
                continue
            err = abs(pos - self._usd_positions[idx])
            errs[idx] = err
            if err > worst_err:
                worst_name, worst_err = name, err

        if None in errs:
            return

        if not self._armed:
            # Converged = every joint within the warn tolerance simultaneously.
            if max(errs) <= self._warn_rad:
                self._armed = True
                self.get_logger().info(
                    f'mount reached its pose (worst joint error '
                    f'{math.degrees(max(errs)):.3f} deg) — deflection peak-hold ARMED; '
                    f'from here gimbal_mount_deflection_deg measures flight deflection '
                    f'(ticket 055 AC4)')
            return

        for idx, err in enumerate(errs):
            if err > self._peak_deflection[idx]:
                self._peak_deflection[idx] = err

        if worst_name is not None and worst_err > self._warn_rad:
            self.get_logger().warn(
                f'mount deflection {math.degrees(worst_err):.2f} deg on {worst_name} '
                f'exceeds {math.degrees(self._warn_rad):.2f} deg — the commanded-joint '
                f'mount is not behaving rigidly (ticket 055 AC4)',
                throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = FixedMountPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
