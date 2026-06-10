from __future__ import annotations

import math

import pytest

from rssi_triangulation.aggregate import AggregatedWifiReading
from rssi_triangulation.locate import (
    PositionReading,
    access_points_relative_to_position,
    apply_floor_origin,
    build_readings_dict,
    match_readings_to_aps,
    smooth_position,
)
from rssi_triangulation.module_config import parse_config_dict
from rssi_triangulation.registry import ApRegistry
from rssi_triangulation.triangulate import PositionEstimate


def test_position_reading_output_keys() -> None:
    assert PositionReading(3.0, 4.0).as_dict() == {
        "location": {"x": 3.0, "y": 4.0, "z": 0.0, "unit": "meters"},
    }


def test_access_points_relative_to_position(sample_config_dict: dict) -> None:
    config = parse_config_dict(sample_config_dict)
    position = PositionReading(1.0, 2.0)
    matched = [("AP-B", -55.0, None), ("AP-A", -70.0, None)]
    aps = access_points_relative_to_position(position, matched, config)
    assert len(aps) == 2
    assert aps[0]["name"] == "AP-B"
    assert aps[0]["rssi"] == -55.0
    assert aps[0]["bssid"] == "aa:bb:cc:dd:ee:02"
    # AP-B at (9, -2) in reading frame → offset (8, -4) from (1, 2)
    assert aps[0]["x"] == pytest.approx(8.0)
    assert aps[0]["y"] == pytest.approx(-4.0)
    assert aps[0]["range_m"] == pytest.approx(math.hypot(8.0, 4.0), abs=0.01)
    assert aps[0]["unit"] == "meters"


def test_access_points_include_z_offset(sample_config_dict: dict) -> None:
    config_dict = {
        **sample_config_dict,
        "floor_plan": {
            **sample_config_dict["floor_plan"],
            "device_z_m": 1.5,
            "access_point_z_m": 3.0,
        },
    }
    config = parse_config_dict(config_dict)
    position = PositionReading(1.0, 2.0, z_m=1.5)
    matched = [("AP-A", -60.0, None)]
    aps = access_points_relative_to_position(position, matched, config)
    assert aps[0]["z"] == pytest.approx(1.5)


def test_build_readings_dict_includes_access_points(sample_config_dict: dict) -> None:
    config = parse_config_dict(sample_config_dict)
    position = PositionReading(0.0, 0.0)
    matched = [("AP-A", -60.0, None)]
    payload = build_readings_dict(position, matched, config)
    assert "location" in payload
    assert len(payload["access_points"]) == 1
    assert payload["access_points"][0]["name"] == "AP-A"


def test_apply_floor_origin() -> None:
    est = PositionEstimate(
        x_m=10.0,
        y_m=20.0,
        unit="m",
        method="weighted_centroid",
        anchor_count=2,
        anchors_used=("a", "b"),
    )
    pos = apply_floor_origin(est, x_origin_m=1.0, y_origin_m=2.0)
    assert pos.x_m == 9.0
    assert pos.y_m == 18.0


def test_fingerprint_ground_level_z_preserved() -> None:
    """z_m=0 is valid floor height; must not be treated as unset."""
    from rssi_triangulation.fingerprint import FingerprintMatch

    fp = FingerprintMatch(
        x_m=5.0,
        y_m=10.0,
        z_m=0.0,
        label="floor",
        distance_db=1.0,
        common_aps=3,
        k=1,
        neighbors=("floor",),
    )
    pos = PositionReading(x_m=fp.x_m, y_m=fp.y_m, z_m=fp.z_m)
    assert pos.z_m == 0.0


def test_smooth_position_applies_alpha_to_z() -> None:
    previous = PositionReading(0.0, 0.0, z_m=1.0)
    current = PositionReading(10.0, 0.0, z_m=3.0)
    smoothed = smooth_position(previous, current, alpha=0.5, max_step_m=None)
    assert smoothed.x_m == 5.0
    assert smoothed.z_m == pytest.approx(2.0)


def test_smooth_position_limits_z_step() -> None:
    previous = PositionReading(0.0, 0.0, z_m=0.0)
    current = PositionReading(0.0, 0.0, z_m=5.0)
    smoothed = smooth_position(previous, current, alpha=1.0, max_step_m=1.0)
    assert smoothed.z_m == pytest.approx(1.0)


def test_smooth_position_first_reading_unchanged() -> None:
    current = PositionReading(10.0, 20.0)
    assert smooth_position(None, current) == current


def test_smooth_position_applies_alpha_and_step_limit() -> None:
    previous = PositionReading(0.0, 0.0)
    current = PositionReading(10.0, 0.0)
    smoothed = smooth_position(previous, current, alpha=0.5, max_step_m=1.0)
    assert smoothed.x_m == 1.0
    assert smoothed.y_m == 0.0


def test_smooth_position_can_disable_step_limit() -> None:
    previous = PositionReading(0.0, 0.0)
    current = PositionReading(10.0, 0.0)
    smoothed = smooth_position(previous, current, alpha=0.5, max_step_m=None)
    assert smoothed.x_m == 5.0
    assert smoothed.y_m == 0.0


def test_match_readings_to_aps_keeps_strongest_per_name(sample_registry: ApRegistry) -> None:
    readings = [
        AggregatedWifiReading(
            bssid="aa:bb:cc:dd:ee:01",
            ssid="Viam-5G",
            rssi_dbm=-70.0,
            rssi_std_dbm=None,
            sample_count=1,
            frequency_mhz=5180,
            backend="iw",
        ),
        AggregatedWifiReading(
            bssid="aa:bb:cc:dd:ee:01",
            ssid="Viam-5G",
            rssi_dbm=-55.0,
            rssi_std_dbm=None,
            sample_count=1,
            frequency_mhz=5180,
            backend="iw",
        ),
        AggregatedWifiReading(
            bssid="aa:bb:cc:dd:ee:02",
            ssid="Viam-5G",
            rssi_dbm=-60.0,
            rssi_std_dbm=None,
            sample_count=1,
            frequency_mhz=5180,
            backend="iw",
        ),
    ]
    matched = match_readings_to_aps(readings, sample_registry)
    assert len(matched) == 2
    by_name = {name: rssi for name, rssi, _ in matched}
    assert by_name["AP-A"] == -55.0
    assert by_name["AP-B"] == -60.0


def test_match_readings_unknown_bssid_ignored(sample_registry: ApRegistry) -> None:
    readings = [
        AggregatedWifiReading(
            bssid="ff:ff:ff:ff:ff:ff",
            ssid="Viam-5G",
            rssi_dbm=-40.0,
            rssi_std_dbm=None,
            sample_count=1,
            frequency_mhz=None,
            backend="iw",
        ),
    ]
    assert match_readings_to_aps(readings, sample_registry) == []


def test_match_readings_filters_low_sample_count(sample_registry: ApRegistry) -> None:
    readings = [
        AggregatedWifiReading(
            bssid="aa:bb:cc:dd:ee:01",
            ssid="Viam-5G",
            rssi_dbm=-50.0,
            rssi_std_dbm=None,
            sample_count=1,
            frequency_mhz=5180,
            backend="iw",
        ),
        AggregatedWifiReading(
            bssid="aa:bb:cc:dd:ee:02",
            ssid="Viam-5G",
            rssi_dbm=-60.0,
            rssi_std_dbm=None,
            sample_count=2,
            frequency_mhz=5180,
            backend="iw",
        ),
    ]
    matched = match_readings_to_aps(readings, sample_registry, min_sample_count=2)
    assert matched == [("AP-B", -60.0, 5180)]
