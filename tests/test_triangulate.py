from __future__ import annotations

import math

import pytest

from rssi_triangulation.registry import AccessPoint, ApRegistry
from rssi_triangulation.triangulate import (
    Anchor,
    anchors_from_readings,
    distance_3d_m,
    estimate_position,
    estimate_weighted_centroid,
    filter_anchors,
)


def test_weighted_centroid_midpoint(triangle_registry: ApRegistry) -> None:
    """Equal RSSI at symmetric anchors on x-axis → midpoint."""
    readings = [
        ("left", -60.0, None),
        ("right", -60.0, None),
    ]
    est = estimate_position(
        triangle_registry,
        readings,
        min_anchors=2,
        max_rssi_delta_db=None,
    )
    assert est is not None
    assert est.method == "weighted_centroid"
    assert est.x_m == pytest.approx(5.0, abs=0.01)
    assert est.y_m == pytest.approx(0.0, abs=0.01)


def test_stronger_ap_pulls_centroid(triangle_registry: ApRegistry) -> None:
    readings = [
        ("left", -50.0, None),
        ("right", -80.0, None),
        ("top", -80.0, None),
    ]
    est = estimate_position(
        triangle_registry,
        readings,
        min_anchors=2,
        max_rssi_delta_db=None,
    )
    assert est is not None
    assert est.x_m < 5.0
    assert est.y_m < 5.0


def test_weight_temperature_softens_pull(triangle_registry: ApRegistry) -> None:
    """Higher temperature flattens weights so the strongest AP pulls less."""
    readings = [
        ("left", -50.0, None),
        ("right", -80.0, None),
        ("top", -80.0, None),
    ]
    sharp = estimate_position(
        triangle_registry,
        readings,
        min_anchors=2,
        max_rssi_delta_db=None,
        weight_temperature=1.0,
    )
    soft = estimate_position(
        triangle_registry,
        readings,
        min_anchors=2,
        max_rssi_delta_db=None,
        weight_temperature=4.0,
    )
    assert sharp is not None and soft is not None
    # "left" anchor is at the origin; softer weighting stays farther from it.
    assert soft.x_m > sharp.x_m
    assert soft.y_m > sharp.y_m


def test_weight_temperature_preserves_symmetry(triangle_registry: ApRegistry) -> None:
    """Equal RSSI → midpoint regardless of temperature."""
    readings = [("left", -60.0, None), ("right", -60.0, None)]
    est = estimate_position(
        triangle_registry,
        readings,
        min_anchors=2,
        max_rssi_delta_db=None,
        weight_temperature=3.0,
    )
    assert est is not None
    assert est.x_m == pytest.approx(5.0, abs=0.01)
    assert est.y_m == pytest.approx(0.0, abs=0.01)


def test_filter_anchors_drops_weak_outliers() -> None:
    anchors = [
        Anchor("a", 0, 0, -50),
        Anchor("b", 10, 0, -85),
    ]
    kept = filter_anchors(anchors, max_delta_db=30.0)
    assert len(kept) == 1
    assert kept[0].ap_name == "a"


def test_estimate_position_insufficient_anchors(triangle_registry: ApRegistry) -> None:
    readings = [("left", -60.0, None)]
    assert estimate_position(triangle_registry, readings, min_anchors=2) is None


def test_nearest_ap_single_anchor(triangle_registry: ApRegistry) -> None:
    anchors = anchors_from_readings(triangle_registry, [("right", -55.0, None)])
    est = estimate_weighted_centroid(anchors)
    assert est is not None
    assert est.method == "nearest_ap"
    assert est.x_m == 10.0
    assert est.y_m == 0.0


def test_distance_3d_is_vertical_when_directly_under_ap() -> None:
    anchor = Anchor("above", 0.0, 0.0, -50.0, z_m=2.5)
    assert distance_3d_m(0.0, 0.0, 0.0, anchor) == pytest.approx(2.5)


def _ceiling_ap_registry() -> ApRegistry:
    return ApRegistry(
        scan_ssid="TestNet",
        access_points=(
            AccessPoint(
                ap_name="above",
                x_m=10.0,
                y_m=20.0,
                bssid="aa:bb:cc:00:00:01",
                z_m=2.5,
            ),
            AccessPoint(
                ap_name="far",
                x_m=40.0,
                y_m=20.0,
                bssid="aa:bb:cc:00:00:02",
                z_m=2.5,
            ),
        ),
    )


def test_under_ceiling_ap_single_anchor_is_exact() -> None:
    registry = _ceiling_ap_registry()
    tx_power = -40.0
    path_loss_n = 2.5
    rssi_above = tx_power - 10 * path_loss_n * math.log10(2.5)
    est = estimate_position(
        registry,
        [("above", rssi_above, None)],
        min_anchors=1,
        device_z_m=0.0,
        tx_power_dbm=tx_power,
        path_loss_n=path_loss_n,
    )
    assert est is not None
    assert est.method == "nearest_ap"
    assert est.x_m == pytest.approx(10.0)
    assert est.y_m == pytest.approx(20.0)


def test_under_ceiling_ap_stays_at_ap_xy() -> None:
    """3D slant range keeps x/y at the AP when range is mostly vertical."""
    registry = _ceiling_ap_registry()
    tx_power = -40.0
    path_loss_n = 2.5
    rssi_above = tx_power - 10 * path_loss_n * math.log10(2.5)
    readings = [
        ("above", rssi_above, None),
        ("far", rssi_above - 35.0, None),
    ]
    est = estimate_position(
        registry,
        readings,
        min_anchors=2,
        device_z_m=0.0,
        tx_power_dbm=tx_power,
        path_loss_n=path_loss_n,
        max_rssi_delta_db=None,
        min_rssi_dbm=-100.0,
    )
    assert est is not None
    assert est.method == "weighted_centroid_3d"
    assert est.x_m == pytest.approx(10.0, abs=0.5)
    assert est.y_m == pytest.approx(20.0, abs=0.5)


def test_without_vertical_geometry_skips_3d_refine(triangle_registry: ApRegistry) -> None:
    readings = [("left", -60.0, None), ("right", -60.0, None)]
    est = estimate_position(
        triangle_registry,
        readings,
        min_anchors=2,
        device_z_m=0.0,
        max_rssi_delta_db=None,
    )
    assert est is not None
    assert est.method == "weighted_centroid"
