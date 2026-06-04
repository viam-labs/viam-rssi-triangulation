from __future__ import annotations

import pytest

from rssi_triangulation.registry import ApRegistry
from rssi_triangulation.triangulate import (
    Anchor,
    anchors_from_readings,
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
