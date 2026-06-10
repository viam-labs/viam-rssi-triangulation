from __future__ import annotations

import math

from rssi_triangulation.fusion import (
    MotionDelta,
    PositionFilter,
    measurement_var_from_fix,
    slam_pose_delta,
)


def test_first_update_seeds_to_measurement() -> None:
    f = PositionFilter()
    assert not f.initialized
    assert f.update(5.0, 7.0) is True
    assert f.position == (5.0, 7.0)


def test_stationary_heavily_smooths_noise() -> None:
    """With no motion the estimate barely follows a jumpy fix."""
    f = PositionFilter(process_noise_m=0.1, measurement_noise_m=3.0)
    f.update(0.0, 0.0)
    still = MotionDelta(speed_mps=0.0, is_moving=False)
    noisy = [(2.0, -2.0), (-2.5, 2.0), (3.0, 1.5), (-2.0, -1.0)]
    for x, y in noisy:
        f.predict(still, dt_s=1.0)
        f.update(x, y)
    fx, fy = f.position
    assert abs(fx) < 1.0
    assert abs(fy) < 1.0


def test_moving_tracks_measurements_faster() -> None:
    """Higher speed inflates process noise so the filter follows fixes."""
    stationary = PositionFilter(
        process_noise_m=0.1, speed_scale=1.0, max_innovation_m=100.0
    )
    moving = PositionFilter(
        process_noise_m=0.1, speed_scale=1.0, max_innovation_m=100.0
    )
    target = (10.0, 0.0)
    stationary.update(0.0, 0.0)
    moving.update(0.0, 0.0)
    for _ in range(3):
        stationary.predict(MotionDelta(speed_mps=0.0, is_moving=False), dt_s=1.0)
        stationary.update(*target)
        moving.predict(MotionDelta(speed_mps=2.0, is_moving=True), dt_s=1.0)
        moving.update(*target)
    assert moving.position[0] > stationary.position[0]


def test_directional_prediction_shifts_estimate() -> None:
    f = PositionFilter(process_noise_m=0.1)
    f.update(0.0, 0.0)
    f.predict(MotionDelta(dx_m=3.0, dy_m=4.0, has_direction=True, speed_mps=5.0), dt_s=1.0)
    # Before any correction the prediction has moved the mean by the delta.
    assert f.position == (3.0, 4.0)


def test_innovation_gate_rejects_outliers() -> None:
    f = PositionFilter(max_innovation_m=5.0)
    f.update(0.0, 0.0)
    f.predict(MotionDelta(is_moving=False), dt_s=1.0)
    accepted = f.update(50.0, 0.0, max_innovation_m=5.0)
    assert accepted is False
    assert f.position[0] < 1.0


def test_gate_reseeds_after_persistent_disagreement() -> None:
    f = PositionFilter(max_innovation_m=5.0, max_consecutive_rejects=3)
    f.update(0.0, 0.0)
    accepted = []
    for _ in range(3):
        f.predict(MotionDelta(is_moving=False), dt_s=1.0)
        accepted.append(f.update(50.0, 0.0, max_innovation_m=5.0))
    assert accepted == [False, False, True]
    assert f.position == (50.0, 0.0)


def test_slam_pose_delta_converts_mm_to_m() -> None:
    dx, dy = slam_pose_delta((1000.0, 2000.0), (1500.0, 2000.0))
    assert math.isclose(dx, 0.5)
    assert math.isclose(dy, 0.0)


def test_slam_pose_delta_applies_yaw_offset() -> None:
    dx, dy = slam_pose_delta((0.0, 0.0), (1000.0, 0.0), yaw_offset_deg=90.0)
    assert math.isclose(dx, 0.0, abs_tol=1e-9)
    assert math.isclose(dy, 1.0, abs_tol=1e-9)


def test_measurement_var_tightens_with_more_anchors() -> None:
    few = measurement_var_from_fix(base_noise_m=3.0, anchor_count=3)
    many = measurement_var_from_fix(base_noise_m=3.0, anchor_count=8)
    assert many < few


def test_measurement_var_tightens_with_fingerprint_confidence() -> None:
    no_fp = measurement_var_from_fix(base_noise_m=3.0, anchor_count=4)
    with_fp = measurement_var_from_fix(
        base_noise_m=3.0, anchor_count=4, fp_blend_weight=1.0
    )
    assert with_fp < no_fp
