"""Regression tests for LOS rate controller limit handling."""

import math

from gimbal_stabilizer.los_rate_controller import LOSRateController


def test_qdot_zeroed_when_lower_limit_saturated_outward():
    target, qdot = LOSRateController._clamp_target_and_qdot(
        actual=math.radians(-100.0),
        qdot=-2.0,
        dt=0.01,
        limits=(math.radians(-90.0), math.radians(90.0)),
    )

    assert target == math.radians(-90.0)
    assert qdot == 0.0


def test_qdot_zeroed_when_upper_limit_saturated_outward():
    target, qdot = LOSRateController._clamp_target_and_qdot(
        actual=math.radians(100.0),
        qdot=2.0,
        dt=0.01,
        limits=(math.radians(-90.0), math.radians(90.0)),
    )

    assert target == math.radians(90.0)
    assert qdot == 0.0


def test_qdot_preserved_when_limit_saturated_inward():
    target, qdot = LOSRateController._clamp_target_and_qdot(
        actual=math.radians(-100.0),
        qdot=2.0,
        dt=0.01,
        limits=(math.radians(-90.0), math.radians(90.0)),
    )

    assert target == math.radians(-90.0)
    assert qdot == 2.0


def test_qdot_preserved_inside_limits():
    target, qdot = LOSRateController._clamp_target_and_qdot(
        actual=0.0,
        qdot=2.0,
        dt=0.01,
        limits=(math.radians(-90.0), math.radians(90.0)),
    )

    assert target == 0.02
    assert qdot == 2.0


def test_out_of_range_candidate_accepted_when_violation_decreases():
    accepted = LOSRateController._candidate_reduces_limit_violation(
        current=math.radians(-45.0),
        candidate=math.radians(-40.0),
        limits=(math.radians(-25.0), math.radians(45.0)),
    )

    assert accepted


def test_out_of_range_candidate_rejected_when_violation_increases():
    accepted = LOSRateController._candidate_reduces_limit_violation(
        current=math.radians(-45.0),
        candidate=math.radians(-50.0),
        limits=(math.radians(-25.0), math.radians(45.0)),
    )

    assert not accepted
