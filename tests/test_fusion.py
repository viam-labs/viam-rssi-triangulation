from __future__ import annotations

import math

import pytest

from rssi_triangulation.fusion import (
    MotionDelta,
    PositionFilter,
    SignalFix,
    _CHI2_2DOF_95,
    _inv_2x2,
    _mat_mul,
    _mat_T,
    measurement_var_from_fix,
    slam_pose_delta,
)


# ---------------------------------------------------------------------------
# Matrix helper tests
# ---------------------------------------------------------------------------


def test_mat_mul_identity() -> None:
    I = [[1.0, 0.0], [0.0, 1.0]]
    A = [[3.0, 4.0], [5.0, 6.0]]
    assert _mat_mul(I, A) == A
    assert _mat_mul(A, I) == A


def test_mat_T_roundtrip() -> None:
    A = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]
    assert _mat_T(_mat_T(A)) == A


def test_inv_2x2_identity() -> None:
    I = [[1.0, 0.0], [0.0, 1.0]]
    inv = _inv_2x2(I)
    assert math.isclose(inv[0][0], 1.0)
    assert math.isclose(inv[1][1], 1.0)
    assert math.isclose(inv[0][1], 0.0)
    assert math.isclose(inv[1][0], 0.0)


def test_inv_2x2_known() -> None:
    M = [[4.0, 7.0], [2.0, 6.0]]
    inv = _inv_2x2(M)
    # M @ inv should equal identity
    prod = _mat_mul(M, inv)
    assert math.isclose(prod[0][0], 1.0, abs_tol=1e-10)
    assert math.isclose(prod[0][1], 0.0, abs_tol=1e-10)
    assert math.isclose(prod[1][0], 0.0, abs_tol=1e-10)
    assert math.isclose(prod[1][1], 1.0, abs_tol=1e-10)


# ---------------------------------------------------------------------------
# PositionFilter: initialisation and basic behaviour
# ---------------------------------------------------------------------------


def test_first_update_seeds_to_measurement() -> None:
    f = PositionFilter()
    assert not f.initialized
    assert f.update(5.0, 7.0) is True
    assert f.position == (5.0, 7.0)


def test_reset_clears_state() -> None:
    f = PositionFilter()
    f.update(1.0, 2.0)
    f.reset()
    assert not f.initialized
    assert f.position is None


def test_predict_before_init_is_noop() -> None:
    f = PositionFilter()
    f.predict(MotionDelta(), dt_s=1.0)
    assert not f.initialized


# ---------------------------------------------------------------------------
# Predict step
# ---------------------------------------------------------------------------


def test_directional_prediction_shifts_estimate() -> None:
    f = PositionFilter(process_noise_m=0.1)
    f.update(0.0, 0.0)
    f.predict(MotionDelta(dx_m=3.0, dy_m=4.0, has_direction=True, speed_mps=5.0), dt_s=1.0)
    assert f.position == (3.0, 4.0)


def test_velocity_prediction_without_direction_uses_state() -> None:
    """When has_direction=False the EKF propagates via F(dt): pos += vel*dt."""
    f = PositionFilter()
    # Seed and inject a known velocity state via update + directional predict
    f.update(0.0, 0.0)
    # Force vx=1, vy=0 via IMU delta
    f.predict(
        MotionDelta(dx_m=1.0, dy_m=0.0, vx_m=1.0, vy_m=0.0, has_direction=True),
        dt_s=1.0,
    )
    assert math.isclose(f.position[0], 1.0)
    # Now predict without direction — F propagates velocity states
    f.predict(MotionDelta(has_direction=False, speed_mps=1.0), dt_s=1.0)
    # Position should advance by vx * dt ≈ 1.0 * 1.0
    assert f.position[0] > 1.0


def test_imu_velocity_overrides_velocity_states() -> None:
    """When MotionDelta carries vx_m/vy_m the velocity states are pinned."""
    f = PositionFilter()
    f.update(0.0, 0.0)
    f.predict(
        MotionDelta(dx_m=2.0, dy_m=0.0, vx_m=2.0, vy_m=0.5, has_direction=True),
        dt_s=1.0,
    )
    # Subsequent no-motion predict propagates via stored velocity
    f.predict(MotionDelta(has_direction=False), dt_s=1.0)
    x, y = f.position
    assert x > 2.0   # velocity carried forward
    assert y > 0.0


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
    assert abs(fx) < 1.5
    assert abs(fy) < 1.5


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


# ---------------------------------------------------------------------------
# Update step — chi-squared gate
# ---------------------------------------------------------------------------


def test_chi2_gate_accepts_consistent_measurement() -> None:
    """A measurement close to the estimate should pass the chi-squared gate."""
    f = PositionFilter(chi2_threshold=_CHI2_2DOF_95)
    f.update(0.0, 0.0)
    f.predict(MotionDelta(is_moving=False), dt_s=0.5)
    accepted = f.update(0.3, 0.3)
    assert accepted is True


def test_chi2_gate_rejects_large_outlier() -> None:
    """An extreme outlier should be gated even with a lenient Euclidean cap."""
    f = PositionFilter(chi2_threshold=_CHI2_2DOF_95, max_innovation_m=1000.0)
    f.update(0.0, 0.0)
    f.predict(MotionDelta(is_moving=False), dt_s=0.1)
    accepted = f.update(500.0, 0.0)
    assert accepted is False
    assert f.position[0] < 1.0


def test_innovation_gate_rejects_outliers() -> None:
    """Hard Euclidean gate still works alongside chi-squared gate."""
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


def test_mahalanobis_gate_tighter_after_covariance_shrinks() -> None:
    """After many good fixes covariance shrinks, making the gate stricter."""
    f = PositionFilter(chi2_threshold=_CHI2_2DOF_95, max_innovation_m=100.0)
    f.update(0.0, 0.0)
    for _ in range(10):
        f.predict(MotionDelta(is_moving=False), dt_s=0.5)
        f.update(0.0, 0.0)   # repeatedly confirm position (0,0)
    # After covariance has shrunk, a modest offset may be gated
    f.predict(MotionDelta(is_moving=False), dt_s=0.5)
    # A 10 m jump should definitely be gated now
    accepted = f.update(10.0, 0.0)
    assert accepted is False


# ---------------------------------------------------------------------------
# Covariance positivity
# ---------------------------------------------------------------------------


def test_covariance_stays_positive_definite_through_predict_update() -> None:
    """Diagonal of P must remain non-negative after many predict/update cycles."""
    f = PositionFilter()
    f.update(0.0, 0.0)
    for i in range(20):
        f.predict(MotionDelta(speed_mps=0.5, is_moving=True), dt_s=0.1)
        f.update(float(i) * 0.05, 0.0)
    for row in range(4):
        assert f._P[row][row] >= 0.0, f"P[{row}][{row}] went negative"


# ---------------------------------------------------------------------------
# SLAM helper
# ---------------------------------------------------------------------------


def test_slam_pose_delta_converts_mm_to_m() -> None:
    dx, dy = slam_pose_delta((1000.0, 2000.0), (1500.0, 2000.0))
    assert math.isclose(dx, 0.5)
    assert math.isclose(dy, 0.0)


def test_slam_pose_delta_applies_yaw_offset() -> None:
    dx, dy = slam_pose_delta((0.0, 0.0), (1000.0, 0.0), yaw_offset_deg=90.0)
    assert math.isclose(dx, 0.0, abs_tol=1e-9)
    assert math.isclose(dy, 1.0, abs_tol=1e-9)


# ---------------------------------------------------------------------------
# measurement_var_from_fix
# ---------------------------------------------------------------------------


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


# ---------------------------------------------------------------------------
# SignalFix dataclass
# ---------------------------------------------------------------------------


def test_signal_fix_defaults() -> None:
    fix = SignalFix(
        x_m=3.0,
        y_m=4.0,
        measurement_var_m2=9.0,
        source="wifi",
        anchor_count=5,
        method="centroid",
    )
    assert fix.x_m == 3.0
    assert fix.y_m == 4.0
    assert fix.measurement_var_m2 == 9.0
    assert fix.source == "wifi"
    assert fix.anchor_count == 5
    assert fix.method == "centroid"
    assert fix.metadata == {}


def test_signal_fix_metadata_passthrough() -> None:
    meta = {"backend": "nmcli", "scans": 3}
    fix = SignalFix(
        x_m=1.0,
        y_m=2.0,
        measurement_var_m2=4.0,
        source="wifi",
        anchor_count=3,
        method="centroid+fp",
        metadata=meta,
    )
    assert fix.metadata["backend"] == "nmcli"
    assert fix.metadata["scans"] == 3


def test_signal_fix_ble_source() -> None:
    fix = SignalFix(
        x_m=5.0,
        y_m=7.0,
        measurement_var_m2=6.0,
        source="ble",
        anchor_count=3,
        method="ble-trilateration",
        metadata={"beacon_count": 3},
    )
    assert fix.source == "ble"
    assert fix.metadata["beacon_count"] == 3


# ---------------------------------------------------------------------------
# Multi-source EKF: dual sequential updates in a single predict/update cycle
# ---------------------------------------------------------------------------


def test_dual_source_both_updates_applied() -> None:
    """WiFi and BLE fixes applied sequentially in one cycle should both shift the state."""
    f = PositionFilter(process_noise_m=0.5, max_innovation_m=50.0)
    f.update(0.0, 0.0)

    f.predict(MotionDelta(is_moving=False), dt_s=1.0)

    wifi_fix = SignalFix(
        x_m=3.0, y_m=0.0, measurement_var_m2=9.0, source="wifi", anchor_count=4, method="centroid"
    )
    ble_fix = SignalFix(
        x_m=3.5, y_m=0.0, measurement_var_m2=6.0, source="ble", anchor_count=3, method="ble-trilateration"
    )

    for fix in [wifi_fix, ble_fix]:
        f.update(fix.x_m, fix.y_m, measurement_var_m2=fix.measurement_var_m2)

    x, _ = f.position
    # After two updates from x=3 and x=3.5 the estimate should be between 0 and 3.5
    assert x > 0.0
    assert x < 4.0


def test_dual_source_tighter_variance_pulls_more() -> None:
    """The source with lower measurement variance should attract the estimate more.

    Starting from x=3, two updates at x=2 and x=4 are given with swapped
    variances.  The tightly-weighted source dominates, so whichever measurement
    carries the lower variance pulls the final estimate toward itself.
    Note: measurements must stay within the chi-squared gate so neither update
    is rejected; the values here are chosen to ensure that.
    """
    f_tight_low = PositionFilter(max_innovation_m=100.0)
    f_tight_high = PositionFilter(max_innovation_m=100.0)

    for f in (f_tight_low, f_tight_high):
        f.update(3.0, 0.0)
        f.predict(MotionDelta(is_moving=False), dt_s=0.1)

    # tight at x=2, loose at x=4 → estimate pulled toward x=2 (lower)
    f_tight_low.update(2.0, 0.0, measurement_var_m2=0.25)
    f_tight_low.update(4.0, 0.0, measurement_var_m2=25.0)

    # loose at x=2, tight at x=4 → estimate pulled toward x=4 (higher)
    f_tight_high.update(2.0, 0.0, measurement_var_m2=25.0)
    f_tight_high.update(4.0, 0.0, measurement_var_m2=0.25)

    assert f_tight_low.position[0] < f_tight_high.position[0]


def test_ble_only_ekf_cold_start_convergence() -> None:
    """EKF initialized from BLE-only fixes (no WiFi) should converge to the true position."""
    f = PositionFilter(
        process_noise_m=0.5,
        measurement_noise_m=2.5,
        max_innovation_m=20.0,
    )
    true_x, true_y = 7.0, 4.0
    still = MotionDelta(speed_mps=0.0, is_moving=False)

    for _ in range(8):
        f.predict(still, dt_s=1.0)
        f.update(true_x, true_y, measurement_var_m2=6.0)

    x, y = f.position
    assert abs(x - true_x) < 1.0, f"x error {abs(x - true_x):.2f} m"
    assert abs(y - true_y) < 1.0, f"y error {abs(y - true_y):.2f} m"


def test_ble_only_ekf_reseeds_after_relocation() -> None:
    """When BLE is the only source, the re-seed logic still recovers from a large jump."""
    f = PositionFilter(max_innovation_m=3.0, max_consecutive_rejects=3)
    f.update(0.0, 0.0)

    # Simulate device relocating far away — filter should re-seed after 3 rejects
    accepted = []
    for _ in range(3):
        f.predict(MotionDelta(is_moving=False), dt_s=1.0)
        accepted.append(f.update(20.0, 20.0, measurement_var_m2=6.0))

    assert accepted[-1] is True        # re-seed triggered on the 3rd attempt
    assert f.position == (20.0, 20.0)


def test_wifi_only_behaviour_unchanged() -> None:
    """WiFi-only path (no BLE updates) still produces correct EKF output."""
    f = PositionFilter(process_noise_m=0.2, max_innovation_m=50.0)
    target = (5.0, 3.0)

    f.update(*target)
    for _ in range(5):
        f.predict(MotionDelta(is_moving=False), dt_s=0.5)
        f.update(*target, measurement_var_m2=9.0)

    x, y = f.position
    assert abs(x - target[0]) < 0.5
    assert abs(y - target[1]) < 0.5
