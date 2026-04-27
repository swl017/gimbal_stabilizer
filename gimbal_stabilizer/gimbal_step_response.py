#!/usr/bin/env python3
"""Live step-response logger for the gimbal_stabilizer.

Drives the running stabilizer through the same scenarios as the Isaac Lab
numerical diagnostic and writes one CSV per scenario, sim-time-stamped, for
post-hoc comparison against
iris_ma6/controller/sysid_output/gimbal/results/.

All timing is sim-time-based (driven by /clock), not wall-time. This ensures
each scenario actually runs for `duration` seconds of *simulated* time, even
if the simulation is running below real-time (which is the common case at
high physics rates).

Usage (in your live SITL stack with IsaacSim + PX4 + los_rate_controller up):
  ros2 run gimbal_stabilizer gimbal_step_response \\
      --ros-args -r __ns:=/px4_1 \\
      -p out_dir:=/tmp/gimbal_sitl_step \\
      -p duration_sim:=2.0

The drone must be hovering (or disarmed in a steady pose). Body-rate
disturbance scenarios are not exercised by this test — those would have to
come from the FCU, not the gimbal_cmd_los_rate topic.
"""

import csv
import math
import os
import time

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy,
)
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState


JOINT_NAMES = ['yaw_joint', 'roll_joint', 'pitch_joint']

# (name, description, yaw_rate_rad_s, pitch_rate_rad_s, t_on_sim)
# t_on_sim is in sim-seconds.
SCENARIOS = [
    ("S1_static_hold",
     "Zero command for the duration",
     0.0, 0.0, 0.0),
    ("S2_yaw_slew_static",
     "+30 deg/s azimuth-rate step at t_sim=0.1",
     math.radians(30.0), 0.0, 0.1),
    ("S3_pitch_slew_static",
     "+20 deg/s elevation-rate step at t_sim=0.1",
     0.0, math.radians(20.0), 0.1),
    ("S4_neg_yaw_slew",
     "-30 deg/s azimuth-rate step at t_sim=0.1",
     -math.radians(30.0), 0.0, 0.1),
    ("S5_neg_pitch_slew",
     "-20 deg/s elevation-rate step at t_sim=0.1",
     0.0, -math.radians(20.0), 0.1),
    ("S6_combined_slew",
     "+15 deg/s yaw and +10 deg/s pitch at t_sim=0.1",
     math.radians(15.0), math.radians(10.0), 0.1),
]


class StepResponseLogger(Node):
    def __init__(self):
        super().__init__('gimbal_step_response')

        self.declare_parameter('out_dir', '/tmp/gimbal_sitl_step')
        self.declare_parameter('duration_sim', 2.0)
        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('settle_sim', 0.5)
        self.declare_parameter('imu_topic', 'mavros/imu/data')
        self.declare_parameter('clock_topic', '/clock')

        self.out_dir = self.get_parameter('out_dir').value
        self.duration_sim = float(self.get_parameter('duration_sim').value)
        self.pub_rate = float(self.get_parameter('publish_rate').value)
        self.settle_sim = float(self.get_parameter('settle_sim').value)
        os.makedirs(self.out_dir, exist_ok=True)

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=10)

        self._cmd_pub = self.create_publisher(Vector3, 'gimbal_cmd_los_rate', sensor_qos)
        self.create_subscription(JointState, 'isaac_joint_states',
                                 self._joint_cb, sensor_qos)
        self.create_subscription(Imu, self.get_parameter('imu_topic').value,
                                 self._imu_cb, sensor_qos)
        # /clock is global, not namespaced. Use absolute topic name.
        self.create_subscription(Clock, self.get_parameter('clock_topic').value,
                                 self._clock_cb, sensor_qos)

        self._joint_pos = {n: float('nan') for n in JOINT_NAMES}
        self._joint_t = 0.0
        self._imu_quat = (float('nan'),) * 4
        self._imu_omega = (float('nan'),) * 3
        self._sim_now = float('nan')

        self.get_logger().info(
            f"step-response logger ready. out_dir={self.out_dir}, "
            f"duration_sim={self.duration_sim}s, settle_sim={self.settle_sim}s, "
            f"publish_rate={self.pub_rate}Hz (wall)")

    def _joint_cb(self, msg: JointState):
        for i, n in enumerate(msg.name):
            if n in self._joint_pos:
                self._joint_pos[n] = msg.position[i]
        self._joint_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _imu_cb(self, msg: Imu):
        self._imu_quat = (msg.orientation.w, msg.orientation.x,
                          msg.orientation.y, msg.orientation.z)
        self._imu_omega = (msg.angular_velocity.x, msg.angular_velocity.y,
                           msg.angular_velocity.z)

    def _clock_cb(self, msg: Clock):
        self._sim_now = msg.clock.sec + msg.clock.nanosec * 1e-9

    # ------------------------------------------------------------------
    # Sim-time helpers
    # ------------------------------------------------------------------

    def _spin_until_sim_advances(self, dt_sim_target: float, max_wall: float = 30.0):
        """Spin until sim clock advances by `dt_sim_target` seconds, or wall
        timeout. Returns True if advanced, False on timeout.

        Inside the loop we also publish the *current* command (passed via
        `_pending_cmd`) at the wall publish_rate so the controller never sees
        a stale or absent setpoint.
        """
        wall_t0 = time.time()
        sim_t0 = self._sim_now
        dt_pub = 1.0 / self.pub_rate
        next_pub = time.time()
        while True:
            rclpy.spin_once(self, timeout_sec=0.001)
            if not math.isnan(self._sim_now) and not math.isnan(sim_t0):
                if self._sim_now - sim_t0 >= dt_sim_target:
                    return True
            now = time.time()
            if now - wall_t0 > max_wall:
                self.get_logger().warning(
                    f"timeout waiting for sim to advance "
                    f"{dt_sim_target:.3f}s (wall {now - wall_t0:.1f}s)")
                return False
            # Re-publish current pending command at wall publish_rate
            if now >= next_pub and hasattr(self, '_pending_cmd'):
                self._cmd_pub.publish(self._pending_cmd)
                next_pub = now + dt_pub
            # On first iteration sim_t0 may have been NaN; refresh once we
            # have a clock reading.
            if math.isnan(sim_t0) and not math.isnan(self._sim_now):
                sim_t0 = self._sim_now

    def _set_cmd(self, yaw_rate: float, pitch_rate: float):
        msg = Vector3()
        msg.x = float(yaw_rate)
        msg.y = float(pitch_rate)
        msg.z = 0.0
        self._pending_cmd = msg
        self._cmd_pub.publish(msg)

    def _wait_for_data(self, wall_timeout: float = 10.0):
        """Block until joint_states, IMU, and /clock all have valid readings."""
        t0 = time.time()
        while time.time() - t0 < wall_timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            ok = (
                not any(math.isnan(v) for v in self._joint_pos.values())
                and not any(math.isnan(v) for v in self._imu_quat)
                and not math.isnan(self._sim_now)
            )
            if ok:
                self.get_logger().info(
                    f"  data ready: sim_now={self._sim_now:.3f}, "
                    f"joints={ {k: round(v, 3) for k, v in self._joint_pos.items()} }")
                return True
        return False

    # ------------------------------------------------------------------
    # Scenario runner
    # ------------------------------------------------------------------

    def run_scenario(self, name, desc, yaw_rate, pitch_rate, t_on_sim):
        self.get_logger().info(f"-- {name}: {desc}")

        # Pre-roll: zero command for `settle_sim` sim-seconds
        self._set_cmd(0.0, 0.0)
        if not self._spin_until_sim_advances(self.settle_sim):
            return False

        rows = []
        scenario_sim_t0 = self._sim_now
        wall_t0 = time.time()
        dt_pub = 1.0 / self.pub_rate
        next_pub = time.time()

        # Drive cmd as a function of sim-elapsed; publish at wall-rate to keep
        # the controller's queue fresh.
        applied_cmd_yet = False
        while True:
            rclpy.spin_once(self, timeout_sec=0.001)
            sim_elapsed = self._sim_now - scenario_sim_t0

            if sim_elapsed >= self.duration_sim:
                break

            # Step input: zero before t_on_sim, commanded value after
            if sim_elapsed >= t_on_sim:
                self._set_cmd(yaw_rate, pitch_rate)
                applied_cmd_yet = True
            else:
                self._set_cmd(0.0, 0.0)

            now = time.time()
            if now >= next_pub:
                # _set_cmd already published; just schedule next
                next_pub = now + dt_pub
                # Snapshot a row at the wall publish rate
                rows.append({
                    "t_wall": now - wall_t0,
                    "t_sim": sim_elapsed,
                    "t_joint": self._joint_t,
                    "cmd_y": self._pending_cmd.x, "cmd_p": self._pending_cmd.y,
                    "yaw_joint": self._joint_pos['yaw_joint'],
                    "roll_joint": self._joint_pos['roll_joint'],
                    "pitch_joint": self._joint_pos['pitch_joint'],
                    "imu_qw": self._imu_quat[0], "imu_qx": self._imu_quat[1],
                    "imu_qy": self._imu_quat[2], "imu_qz": self._imu_quat[3],
                    "imu_wx": self._imu_omega[0], "imu_wy": self._imu_omega[1],
                    "imu_wz": self._imu_omega[2],
                })

            # Wall-time guard: if sim isn't advancing (paused?), bail after a
            # generous timeout.
            wall_elapsed = now - wall_t0
            if wall_elapsed > 60.0 and sim_elapsed < 1e-3:
                self.get_logger().error(
                    f"Sim clock not advancing during {name} "
                    f"(wall {wall_elapsed:.1f}s, sim {sim_elapsed:.3f}s). Aborting.")
                return False
            # Generous safety timeout for scenarios with low RTF
            if wall_elapsed > 600.0:
                self.get_logger().error(
                    f"Scenario {name} exceeded 10 minutes wall time. Aborting.")
                return False

        # Reset to zero command after scenario
        self._set_cmd(0.0, 0.0)
        self._spin_until_sim_advances(self.settle_sim)

        path = os.path.join(self.out_dir, f"{name}.csv")
        if not rows:
            self.get_logger().error(f"   no rows captured for {name}")
            return False
        with open(path, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)

        sim_dur = rows[-1]["t_sim"] - rows[0]["t_sim"]
        wall_dur = rows[-1]["t_wall"] - rows[0]["t_wall"]
        rtf = sim_dur / wall_dur if wall_dur > 0 else float('nan')
        self.get_logger().info(
            f"   wrote {path} ({len(rows)} rows, "
            f"sim_dur={sim_dur:.2f}s, wall_dur={wall_dur:.2f}s, RTF={rtf:.2f}, "
            f"final yaw={math.degrees(rows[-1]['yaw_joint']):+.2f}°, "
            f"pitch={math.degrees(rows[-1]['pitch_joint']):+.2f}°, "
            f"roll={math.degrees(rows[-1]['roll_joint']):+.2f}°)")
        return True

    def run_all(self):
        if not self._wait_for_data():
            self.get_logger().error(
                "Timed out waiting for joint_states / IMU / clock. Check that "
                "the namespace is right (-r __ns:=/px4_1), that IsaacSim is "
                "publishing isaac_joint_states, and that /clock is up "
                "(use_sim_time should be true).")
            return False
        all_ok = True
        for s in SCENARIOS:
            if not self.run_scenario(*s):
                all_ok = False
        return all_ok


def main():
    rclpy.init()
    node = StepResponseLogger()
    # Make sure the node uses sim time so the message stamps line up — even
    # though we read /clock manually, this keeps timer-based behaviour aligned.
    node.set_parameters([
        rclpy.parameter.Parameter(
            'use_sim_time', rclpy.parameter.Parameter.Type.BOOL, True),
    ])
    try:
        ok = node.run_all()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    main()
