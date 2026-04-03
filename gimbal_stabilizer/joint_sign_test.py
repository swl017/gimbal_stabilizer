#!/usr/bin/env python3
"""Joint sign convention test for iris_gimbal3.

Systematically commands each gimbal joint to positive/negative values
(with controller STOPPED) and reads back joint state + IMU to determine
the actual physical effect of each joint direction.

Usage:
  1. Stop los_rate_controller
  2. ros2 run gimbal_stabilizer joint_sign_test --ros-args -r __ns:=/px4_1
  3. Read output to determine sign conventions

Requires: isaac_joint_states publishing, mavros/imu/data publishing.
"""

import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Header


YAW_JOINT_OFFSET = math.pi / 2
JOINT_NAMES = ['yaw_joint', 'roll_joint', 'pitch_joint']

# Test angles: zero, then positive and negative for each joint
TEST_ANGLE = math.radians(20)  # 20 degrees


def quat_to_euler(w, x, y, z):
    """wxyz quaternion → (roll, pitch, yaw) in degrees."""
    roll = math.degrees(math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y)))
    sinp = 2*(w*y - z*x)
    pitch = math.degrees(math.asin(max(-1, min(1, sinp))))
    yaw = math.degrees(math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))
    return roll, pitch, yaw


def quat_rotate(q_wxyz, v):
    """Rotate vector v by quaternion q (wxyz)."""
    w = q_wxyz[0]
    u = q_wxyz[1:4]
    t = 2.0 * np.cross(u, v)
    return v + w * t + np.cross(u, t)


class JointSignTest(Node):
    def __init__(self):
        super().__init__('joint_sign_test')

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=10)

        self._cmd_pub = self.create_publisher(JointState, 'isaac_joint_commands', reliable_qos)
        self.create_subscription(JointState, 'isaac_joint_states', self._joint_cb, sensor_qos)
        self.create_subscription(Imu, 'mavros/imu/data', self._imu_cb, sensor_qos)

        self._joint_pos = {}
        self._imu_quat = None
        self._test_started = False

        # Wait for data, then run test
        self.create_timer(1.0, self._check_ready)

    def _joint_cb(self, msg):
        for i, name in enumerate(msg.name):
            if name in JOINT_NAMES:
                self._joint_pos[name] = msg.position[i]

    def _imu_cb(self, msg):
        q = msg.orientation
        self._imu_quat = np.array([q.w, q.x, q.y, q.z])

    def _check_ready(self):
        if self._test_started:
            return
        if len(self._joint_pos) < 3 or self._imu_quat is None:
            self.get_logger().info('Waiting for joint states and IMU...')
            return
        self._test_started = True
        self._run_tests()

    def _publish_joints(self, yaw_raw, roll_raw, pitch_raw):
        """Publish raw joint commands and wait for them to take effect."""
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [float(yaw_raw), float(roll_raw), float(pitch_raw)]
        # Publish multiple times to ensure ArticulationController receives
        for _ in range(20):
            self._cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
        # Wait for physics to settle
        time.sleep(1.0)
        # Update cached state
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)

    def _read_state(self):
        """Read current joint positions and body attitude."""
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)
        joints = {n: math.degrees(self._joint_pos.get(n, 0)) for n in JOINT_NAMES}
        body_rpy = quat_to_euler(*self._imu_quat) if self._imu_quat is not None else (0, 0, 0)

        # Compute camera direction in world frame using FK
        yaw = self._joint_pos.get('yaw_joint', 0) - YAW_JOINT_OFFSET
        roll = self._joint_pos.get('roll_joint', 0)
        pitch = self._joint_pos.get('pitch_joint', 0)

        # FK with positive pitch = down convention (what code assumes)
        cos_p, sin_p = math.cos(pitch), math.sin(pitch)
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        dir_body_code = np.array([cos_p*cos_y, cos_p*sin_y, -sin_p])

        # FK with positive pitch = up convention (alternative)
        dir_body_alt = np.array([cos_p*cos_y, cos_p*sin_y, +sin_p])

        dir_world_code = quat_rotate(self._imu_quat, dir_body_code)
        dir_world_alt = quat_rotate(self._imu_quat, dir_body_alt)

        el_code = math.degrees(math.atan2(dir_world_code[2],
                    math.sqrt(dir_world_code[0]**2 + dir_world_code[1]**2)))
        el_alt = math.degrees(math.atan2(dir_world_alt[2],
                    math.sqrt(dir_world_alt[0]**2 + dir_world_alt[1]**2)))

        return joints, body_rpy, el_code, el_alt

    def _run_tests(self):
        log = self.get_logger().info
        angle_deg = math.degrees(TEST_ANGLE)

        log('=' * 60)
        log('JOINT SIGN CONVENTION TEST')
        log(f'Test angle: ±{angle_deg:.0f} deg')
        log('=' * 60)

        # Baseline: all joints at zero (yaw at offset)
        log('\n--- BASELINE: all joints zero ---')
        self._publish_joints(YAW_JOINT_OFFSET, 0.0, 0.0)
        joints, body_rpy, el_code, el_alt = self._read_state()
        log(f'  Joints: yaw={joints["yaw_joint"]:+.1f} roll={joints["roll_joint"]:+.1f} pitch={joints["pitch_joint"]:+.1f}')
        log(f'  Body RPY: {body_rpy[0]:+.1f}, {body_rpy[1]:+.1f}, {body_rpy[2]:+.1f}')
        log(f'  FK elevation (code -sinP): {el_code:+.1f} deg')
        log(f'  FK elevation (alt  +sinP): {el_alt:+.1f} deg')

        # Test each joint individually
        tests = [
            ('YAW +20', YAW_JOINT_OFFSET + TEST_ANGLE, 0, 0,
             'Positive yaw_joint: camera should pan LEFT (CCW from above)'),
            ('YAW -20', YAW_JOINT_OFFSET - TEST_ANGLE, 0, 0,
             'Positive yaw_joint: camera should pan RIGHT (CW from above)'),
            ('ROLL +20', YAW_JOINT_OFFSET, +TEST_ANGLE, 0,
             'Positive roll_joint: check which way camera tilts'),
            ('ROLL -20', YAW_JOINT_OFFSET, -TEST_ANGLE, 0,
             'Negative roll_joint: check which way camera tilts'),
            ('PITCH +20', YAW_JOINT_OFFSET, 0, +TEST_ANGLE,
             'Positive pitch_joint: check if camera tilts UP or DOWN'),
            ('PITCH -20', YAW_JOINT_OFFSET, 0, -TEST_ANGLE,
             'Negative pitch_joint: check if camera tilts UP or DOWN'),
        ]

        for name, yaw_cmd, roll_cmd, pitch_cmd, description in tests:
            log(f'\n--- {name}: {description} ---')
            self._publish_joints(yaw_cmd, roll_cmd, pitch_cmd)
            joints, body_rpy, el_code, el_alt = self._read_state()
            log(f'  Commanded: yaw={math.degrees(yaw_cmd):+.1f} roll={math.degrees(roll_cmd):+.1f} pitch={math.degrees(pitch_cmd):+.1f}')
            log(f'  Readback:  yaw={joints["yaw_joint"]:+.1f} roll={joints["roll_joint"]:+.1f} pitch={joints["pitch_joint"]:+.1f}')
            log(f'  FK elevation (code -sinP): {el_code:+.1f} deg')
            log(f'  FK elevation (alt  +sinP): {el_alt:+.1f} deg')

        # Reset to zero
        log('\n--- RESET to baseline ---')
        self._publish_joints(YAW_JOINT_OFFSET, 0.0, 0.0)

        log('\n' + '=' * 60)
        log('TEST COMPLETE — check which FK convention matches observed camera tilt')
        log('If PITCH +20 tilts camera DOWN: code convention (-sinP) is correct')
        log('If PITCH +20 tilts camera UP:   code convention is WRONG, need +sinP')
        log('=' * 60)

        raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = JointSignTest()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
