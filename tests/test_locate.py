from __future__ import annotations

from rssi_triangulation.aggregate import AggregatedWifiReading
from rssi_triangulation.locate import (
    PositionReading,
    apply_floor_origin,
    match_readings_to_aps,
    smooth_position,
)
from rssi_triangulation.registry import ApRegistry
from rssi_triangulation.triangulate import PositionEstimate


def test_position_reading_output_keys() -> None:
    assert PositionReading(3.0, 4.0).as_dict() == {
        "position_x_m": 3.0,
        "position_y_m": 4.0,
    }


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
