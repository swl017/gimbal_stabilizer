"""RAL ticket 043 — roll-consistent position-mode IK.

`_world_to_body_joints` must produce gimbal joints whose camera boresight points exactly at the
commanded world LOS, for any body attitude. The superseded decoupled path
(`_world_to_body_angles` + `_compute_stabilizing_roll`) does not — it deflects the boresight
when the joint pitch is non-zero (the RAL031 S9 peer defect). These tests lock the fix and
document the old failure as a regression guard.
"""

import math

from gimbal_stabilizer.los_rate_controller import (
    LOSRateController,
    _quat_from_euler_single,
    _quat_rotate,
)

import numpy as np


def _boresight_world(q_body, yaw, roll, pitch):
    """World-frame camera boresight for the given body attitude and gimbal joints."""
    q_gimbal = LOSRateController._gimbal_joints_to_quat(yaw, roll, pitch)
    fwd_body = _quat_rotate(q_gimbal, np.array([1.0, 0.0, 0.0]))
    return _quat_rotate(q_body, fwd_body)


def _los_world(az, el):
    """Unit world LOS vector for azimuth/elevation (rad)."""
    return np.array([math.cos(el) * math.cos(az),
                     math.cos(el) * math.sin(az),
                     math.sin(el)])


def _angle_deg(u, v):
    """Angle between two vectors, in degrees."""
    c = np.dot(u, v) / (np.linalg.norm(u) * np.linalg.norm(v))
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def test_fix_points_at_los_for_any_body_attitude():
    rng = np.random.default_rng(43)
    worst = 0.0
    for _ in range(2000):
        roll_b, pitch_b, yaw_b = rng.uniform(-math.pi/3, math.pi/3, size=3)
        q_body = _quat_from_euler_single(roll_b, pitch_b, yaw_b)
        az = rng.uniform(-math.pi, math.pi)
        el = rng.uniform(-math.radians(40), math.radians(40))
        yaw, roll, pitch = LOSRateController._world_to_body_joints(az, el, q_body)
        err = _angle_deg(_boresight_world(q_body, yaw, roll, pitch), _los_world(az, el))
        worst = max(worst, err)
    assert worst < 1e-4, f'fix boresight error {worst} deg (must be ~0)'


def test_old_decoupled_ik_deflects_under_tilt():
    # Body tilted like the terminal peer (roll ~33 deg, pitch ~24 deg); horizontal LOS.
    q_body = _quat_from_euler_single(math.radians(33), math.radians(24), math.radians(35))
    az, el = math.radians(120.0), 0.0
    yaw_old, pitch_old = LOSRateController._world_to_body_angles(az, el, q_body)
    roll_old = LOSRateController._compute_stabilizing_roll(yaw_old, q_body)
    err_old = _angle_deg(_boresight_world(q_body, yaw_old, roll_old, pitch_old),
                         _los_world(az, el))
    yaw, roll, pitch = LOSRateController._world_to_body_joints(az, el, q_body)
    err_fix = _angle_deg(_boresight_world(q_body, yaw, roll, pitch), _los_world(az, el))
    assert err_old > 1.0, f'expected the old path to deflect, got {err_old} deg'
    assert err_fix < 1e-6, f'fix must not deflect, got {err_fix} deg'


def test_fix_keeps_horizon_level():
    # The camera 'up' (image vertical) must have no component along body-independent world East
    # once projected: i.e. the fix's cam_up equals the world-up projected orthogonal to LOS.
    q_body = _quat_from_euler_single(math.radians(20), math.radians(-15), math.radians(80))
    az, el = math.radians(-40.0), math.radians(10.0)
    yaw, roll, pitch = LOSRateController._world_to_body_joints(az, el, q_body)
    q_gimbal = LOSRateController._gimbal_joints_to_quat(yaw, roll, pitch)
    cam_up_world = _quat_rotate(q_body, _quat_rotate(q_gimbal, np.array([0.0, 0.0, 1.0])))
    los = _los_world(az, el)
    world_up = np.array([0.0, 0.0, 1.0])
    expected_up = world_up - np.dot(world_up, los) * los
    expected_up /= np.linalg.norm(expected_up)
    assert _angle_deg(cam_up_world, expected_up) < 1e-4
