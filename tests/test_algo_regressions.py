"""Regression tests for estimation bugs found in the field.

The anchor layout and RSSI values mirror a real scan where a miscalibrated
path-loss model pushed the refined estimate off the floor plan (negative y
with every AP at y > 3.8).
"""

from __future__ import annotations

import math

import pytest

from rssi_triangulation.fingerprint import FingerprintStore
from rssi_triangulation.locate import estimate_from_matched
from rssi_triangulation.module_config import parse_config_dict
from rssi_triangulation.registry import AccessPoint, ApRegistry
from rssi_triangulation.triangulate import clamp_to_bounds, estimate_position


FIELD_APS = (
    ("SoA1", 5.22, 4.42, "aa:00:00:00:00:01"),
    ("NoE4", 18.03, 7.26, "aa:00:00:00:00:02"),
    ("EoF3", 15.76, 17.69, "aa:00:00:00:00:03"),
    ("WoStairsY", 22.91, 3.86, "aa:00:00:00:00:04"),
    ("WoLab", 19.96, 28.12, "aa:00:00:00:00:05"),
)
FIELD_READINGS = [
    ("SoA1", -65.6, None),
    ("NoE4", -66.6, None),
    ("EoF3", -79.0, None),
    ("WoStairsY", -84.0, None),
    ("WoLab", -89.0, None),
]


def _field_registry() -> ApRegistry:
    return ApRegistry(
        scan_ssid="Viam-5G",
        access_points=tuple(
            AccessPoint(ap_name=n, x_m=x, y_m=y, bssid=b, z_m=2.44)
            for n, x, y, b in FIELD_APS
        ),
    )


def _field_config_dict() -> dict:
    return {
        "scan_ssid": "Viam-5G",
        "scan_count": 1,
        "floor_plan": {"device_z_m": 0.2, "access_point_z_m": 2.44},
        "access_points": [
            {"name": n, "x_m": x, "y_m": y, "bssid": b}
            for n, x, y, b in FIELD_APS
        ],
    }


def test_clamp_to_bounds_inside_is_noop() -> None:
    aps = _field_registry().access_points
    x, y, clamped = clamp_to_bounds(10.0, 10.0, aps, margin_m=5.0)
    assert (x, y, clamped) == (10.0, 10.0, False)


def test_clamp_to_bounds_limits_runaway() -> None:
    aps = _field_registry().access_points
    x, y, clamped = clamp_to_bounds(3.54, -6.9, aps, margin_m=5.0)
    assert clamped
    assert x == pytest.approx(3.54)
    assert y == pytest.approx(3.86 - 5.0)


def test_miscalibrated_model_cannot_leave_floor() -> None:
    """The original field failure: estimate fled to y=-6.9 with all APs at y>3.8."""
    est = estimate_position(
        _field_registry(),
        FIELD_READINGS,
        min_anchors=3,
        device_z_m=0.2,
        tx_power_dbm=-40.0,
        path_loss_n=2.5,
        max_rssi_delta_db=35.0,
        min_rssi_dbm=-90.0,
    )
    assert est is not None
    assert est.y_m >= 3.86 - 5.0
    assert est.method.endswith("_clamped")


def test_calibrated_model_not_clamped() -> None:
    est = estimate_position(
        _field_registry(),
        FIELD_READINGS,
        min_anchors=3,
        device_z_m=0.2,
        tx_power_dbm=-32.51,
        path_loss_n=3.76,
    )
    assert est is not None
    assert not est.method.endswith("_clamped")
    assert est.y_m > 0


def test_clamp_can_be_disabled() -> None:
    est = estimate_position(
        _field_registry(),
        FIELD_READINGS,
        min_anchors=3,
        device_z_m=0.2,
        tx_power_dbm=-40.0,
        path_loss_n=2.5,
        max_rssi_delta_db=35.0,
        min_rssi_dbm=-90.0,
        clamp_margin_m=None,
    )
    assert est is not None
    assert est.y_m < 0  # unclamped runaway preserved when explicitly disabled


def test_fingerprint_fallback_rejects_garbage_match(tmp_path) -> None:
    """When geometry fails, a fingerprint far beyond max_rms_db must not be returned."""
    config = parse_config_dict(_field_config_dict())
    db = FingerprintStore(tmp_path / "fp.sqlite")
    # Fingerprint whose RSSI shape is wildly different from the live scan.
    db.record(
        "far-corner",
        x_m=1.0,
        y_m=1.0,
        rssi_by_ap={"SoA1": -90.0, "NoE4": -40.0, "EoF3": -41.0},
        scan_count=1,
    )
    # Only 2 anchors pass min_anchors=3 → no centroid → fingerprint-only path.
    matched = [("SoA1", -60.0, None), ("NoE4", -65.0, None), ("EoF3", -89.0, None)]
    with pytest.raises(RuntimeError, match="could not estimate"):
        estimate_from_matched(
            config,
            matched,
            min_anchors=3,
            min_rssi_dbm=-82.0,
            fingerprint_store=db,
            fingerprint_min_common_aps=2,
            fingerprint_max_rms_db=10.0,
        )


def test_wostairsy_delta_flicker_no_longer_flips_position() -> None:
    """Regression: WoStairsY crossing max_rssi_delta_db used to toggle 4 vs 3 anchors."""
    from rssi_triangulation.locate import estimate_from_matched
    from rssi_triangulation.module_config import parse_config_dict

    config = parse_config_dict(_field_config_dict())
    r1 = [
        ("SoA1", -61.61476553996254, None),
        ("NoE4", -69.38523446003747, None),
        ("EoF3", -74.61476553996253, None),
        ("WoStairsY", -81.0358985327497, None),
        ("WoF2", -87.0, None),
        ("MDF", -88.0, None),
        ("WoLab", -88.88019515263734, None),
    ]
    r2 = [
        ("SoA1", -61.43501735426796, None),
        ("NoE4", -69.56498264573204, None),
        ("EoF3", -74.43501735426796, None),
        ("WoStairsY", -81.63333815177829, None),
        ("WoF2", -87.0, None),
        ("MDF", -88.0, None),
        ("WoLab", -88.93839021458217, None),
    ]
    p1, _, _ = estimate_from_matched(config, r1, fingerprint_store=None)
    p2, _, _ = estimate_from_matched(config, r2, fingerprint_store=None)
    assert math.hypot(p2.x_m - p1.x_m, p2.y_m - p1.y_m) < 1.0


def test_fingerprint_fallback_accepts_good_match(tmp_path) -> None:
    config = parse_config_dict(_field_config_dict())
    db = FingerprintStore(tmp_path / "fp.sqlite")
    db.record(
        "desk",
        x_m=6.0,
        y_m=5.0,
        rssi_by_ap={"SoA1": -60.0, "NoE4": -65.0, "EoF3": -89.0},
        scan_count=1,
    )
    matched = [("SoA1", -60.0, None), ("NoE4", -65.0, None), ("EoF3", -89.0, None)]
    position, method_used, fp_match = estimate_from_matched(
        config,
        matched,
        min_anchors=3,
        min_rssi_dbm=-82.0,
        fingerprint_store=db,
        fingerprint_min_common_aps=2,
        fingerprint_max_rms_db=10.0,
    )
    assert method_used == "fingerprint"
    assert position.x_m == pytest.approx(6.0)
    assert position.y_m == pytest.approx(5.0)
