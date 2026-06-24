"""Tests for BLE scanning, buffering, and trilateration."""

from __future__ import annotations

import math
import time

import pytest

from rssi_triangulation.ble_scan import (
    BleReading,
    BleScanBuffer,
    beacon_count_from_snapshot,
    ble_rssi_to_distance_m,
    trilaterate_ble,
)
from rssi_triangulation.module_config import BleBeacon, LocatorConfig, ConfiguredAccessPoint


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


def _make_config(*, width_m: float = 20.0, height_m: float = 20.0) -> LocatorConfig:
    ap = ConfiguredAccessPoint(name="ap1", x_m=0.0, y_m=0.0, z_m=2.4, bssid="aa:bb:cc:dd:ee:ff")
    return LocatorConfig(
        scan_ssid="test",
        scan_count=1,
        x_origin_m=0.0,
        y_origin_m=0.0,
        device_z_m=0.2,
        access_point_z_m=2.4,
        access_points=(ap,),
        width_m=width_m,
        height_m=height_m,
    )


def _beacon(name: str, x: float, y: float, mac: str) -> BleBeacon:
    return BleBeacon(
        name=name, x_m=x, y_m=y, z_m=1.0, mac_address=mac,
        tx_power_dbm=-59.0, path_loss_n=2.0,
    )


def _reading(mac: str, rssi: float, age_s: float = 0.0) -> BleReading:
    return BleReading(
        mac=mac, rssi_dbm=rssi, name="", timestamp_s=time.monotonic() - age_s
    )


# ---------------------------------------------------------------------------
# ble_rssi_to_distance_m
# ---------------------------------------------------------------------------


def test_rssi_at_tx_power_gives_one_meter() -> None:
    """At RSSI == TxPower the path-loss model should return 1 m."""
    d = ble_rssi_to_distance_m(-59.0, -59.0, path_loss_n=2.0)
    assert math.isclose(d, 1.0, rel_tol=1e-9)


def test_rssi_20db_below_tx_gives_ten_meters_n2() -> None:
    """At RSSI = TxPower − 20 dB and n=2, distance = 10 m."""
    d = ble_rssi_to_distance_m(-79.0, -59.0, path_loss_n=2.0)
    assert math.isclose(d, 10.0, rel_tol=1e-6)


def test_distance_increases_with_lower_rssi() -> None:
    d_near = ble_rssi_to_distance_m(-65.0, -59.0, path_loss_n=2.5)
    d_far = ble_rssi_to_distance_m(-75.0, -59.0, path_loss_n=2.5)
    assert d_far > d_near


def test_clamps_to_minimum_distance() -> None:
    d = ble_rssi_to_distance_m(0.0, -59.0, path_loss_n=2.0)
    assert d >= 0.01


# ---------------------------------------------------------------------------
# BleScanBuffer
# ---------------------------------------------------------------------------


def test_buffer_stores_and_retrieves_reading() -> None:
    buf = BleScanBuffer(max_age_s=10.0)
    r = _reading("aa:bb:cc:dd:ee:01", -65.0)
    buf.add(r)
    snap = buf.snapshot()
    assert len(snap) == 1
    assert snap[0].mac == "aa:bb:cc:dd:ee:01"


def test_buffer_keeps_newest_per_mac() -> None:
    buf = BleScanBuffer(max_age_s=10.0)
    now = time.monotonic()
    r1 = BleReading(mac="aa:bb:cc:dd:ee:01", rssi_dbm=-65.0, name="", timestamp_s=now - 2.0)
    r2 = BleReading(mac="aa:bb:cc:dd:ee:01", rssi_dbm=-70.0, name="", timestamp_s=now - 1.0)
    buf.add(r1)
    buf.add(r2)
    snap = buf.snapshot()
    assert len(snap) == 1
    assert snap[0].rssi_dbm == -70.0   # r2 is newer


def test_buffer_drops_stale_entries() -> None:
    buf = BleScanBuffer(max_age_s=1.0)
    old = _reading("aa:bb:cc:dd:ee:01", -65.0, age_s=5.0)   # 5 s old
    fresh = _reading("aa:bb:cc:dd:ee:02", -70.0, age_s=0.0)
    buf.add(old)
    buf.add(fresh)
    snap = buf.snapshot()
    macs = {r.mac for r in snap}
    assert "aa:bb:cc:dd:ee:01" not in macs
    assert "aa:bb:cc:dd:ee:02" in macs


def test_buffer_clear() -> None:
    buf = BleScanBuffer()
    buf.add(_reading("aa:bb:cc:dd:ee:01", -65.0))
    buf.clear()
    assert buf.snapshot() == []


# ---------------------------------------------------------------------------
# beacon_count_from_snapshot
# ---------------------------------------------------------------------------


def test_beacon_count_matches_configured_beacons() -> None:
    beacons = (
        _beacon("b1", 5.0, 0.0, "aa:bb:cc:dd:ee:01"),
        _beacon("b2", 0.0, 5.0, "aa:bb:cc:dd:ee:02"),
    )
    snapshot = [
        _reading("aa:bb:cc:dd:ee:01", -65.0),
        _reading("aa:bb:cc:dd:ee:02", -70.0),
        _reading("ff:ff:ff:ff:ff:ff", -50.0),  # unknown device
    ]
    assert beacon_count_from_snapshot(snapshot, beacons) == 2


def test_beacon_count_respects_rssi_floor() -> None:
    beacons = (_beacon("b1", 5.0, 0.0, "aa:bb:cc:dd:ee:01"),)
    snapshot = [_reading("aa:bb:cc:dd:ee:01", -95.0)]  # below default -90 floor
    assert beacon_count_from_snapshot(snapshot, beacons, min_rssi_dbm=-90.0) == 0


# ---------------------------------------------------------------------------
# trilaterate_ble
# ---------------------------------------------------------------------------


def _make_snapshot_for_beacons(
    config: LocatorConfig,
    beacons: tuple[BleBeacon, ...],
    device_x: float,
    device_y: float,
    device_z: float = 0.2,
) -> list[BleReading]:
    """Generate synthetic readings that should trilaterate to (device_x, device_y)."""
    readings = []
    for b in beacons:
        d = math.sqrt(
            (b.x_m - config.x_origin_m - device_x) ** 2
            + (b.y_m - config.y_origin_m - device_y) ** 2
            + (b.z_m - device_z) ** 2
        )
        rssi = b.tx_power_dbm - 10.0 * b.path_loss_n * math.log10(max(d, 0.01))
        readings.append(_reading(b.mac_address, rssi))
    return readings


def test_trilaterate_returns_none_with_no_visible_beacons() -> None:
    config = _make_config()
    beacons = (_beacon("b1", 5.0, 0.0, "aa:bb:cc:dd:ee:01"),)
    result = trilaterate_ble([], beacons, config)
    assert result is None


def test_trilaterate_returns_none_for_unknown_macs() -> None:
    config = _make_config()
    beacons = (_beacon("b1", 5.0, 0.0, "aa:bb:cc:dd:ee:01"),)
    snapshot = [_reading("ff:ff:ff:ff:ff:ff", -65.0)]
    result = trilaterate_ble(snapshot, beacons, config, prior_x=5.0, prior_y=5.0)
    assert result is None


def test_trilaterate_three_beacons_recovers_position() -> None:
    """Three synthetic readings should trilaterate close to the true position."""
    config = _make_config()
    beacons = (
        _beacon("b1",  0.0,  0.0, "aa:bb:cc:dd:ee:01"),
        _beacon("b2", 20.0,  0.0, "aa:bb:cc:dd:ee:02"),
        _beacon("b3", 10.0, 20.0, "aa:bb:cc:dd:ee:03"),
    )
    true_x, true_y = 8.0, 6.0
    snapshot = _make_snapshot_for_beacons(config, beacons, true_x, true_y)
    result = trilaterate_ble(snapshot, beacons, config, device_z_m=0.2)
    assert result is not None
    x, y = result
    assert abs(x - true_x) < 0.5, f"x error {abs(x - true_x):.2f} m"
    assert abs(y - true_y) < 0.5, f"y error {abs(y - true_y):.2f} m"


def test_trilaterate_two_beacons_succeeds() -> None:
    config = _make_config()
    beacons = (
        _beacon("b1",  0.0,  0.0, "aa:bb:cc:dd:ee:01"),
        _beacon("b2", 20.0,  0.0, "aa:bb:cc:dd:ee:02"),
    )
    snapshot = _make_snapshot_for_beacons(config, beacons, 10.0, 5.0)
    result = trilaterate_ble(snapshot, beacons, config, device_z_m=0.2)
    # Two circles may give two solutions; the function picks the in-floor one
    assert result is not None


def test_trilaterate_single_beacon_uses_prior() -> None:
    """With one beacon the function falls back to the prior direction."""
    config = _make_config()
    beacons = (_beacon("b1", 0.0, 0.0, "aa:bb:cc:dd:ee:01"),)
    # Device is 5 m north of beacon
    snapshot = _make_snapshot_for_beacons(config, beacons, 0.0, 5.0)
    result = trilaterate_ble(
        snapshot, beacons, config, device_z_m=0.2, prior_x=0.0, prior_y=5.0
    )
    assert result is not None
    x, y = result
    # Should place device in the north direction from beacon
    assert y > 0.0


def test_trilaterate_below_rssi_floor_filtered() -> None:
    config = _make_config()
    beacons = (
        _beacon("b1",  0.0, 0.0, "aa:bb:cc:dd:ee:01"),
        _beacon("b2", 10.0, 0.0, "aa:bb:cc:dd:ee:02"),
        _beacon("b3",  5.0, 8.0, "aa:bb:cc:dd:ee:03"),
    )
    # Put all readings well below the RSSI floor
    snapshot = [_reading(b.mac_address, -100.0) for b in beacons]
    result = trilaterate_ble(snapshot, beacons, config, min_rssi_dbm=-90.0)
    assert result is None


def test_trilaterate_with_floor_origin_offset() -> None:
    """Beacon absolute positions are correctly mapped to the reading frame."""
    config = LocatorConfig(
        scan_ssid="test",
        scan_count=1,
        x_origin_m=5.0,
        y_origin_m=3.0,
        device_z_m=0.2,
        access_point_z_m=2.4,
        access_points=(
            ConfiguredAccessPoint(
                name="ap1", x_m=5.0, y_m=3.0, z_m=2.4, bssid="aa:bb:cc:dd:ee:ff"
            ),
        ),
        width_m=20.0,
        height_m=20.0,
    )
    # Beacons in absolute floor coords; device at reading-frame (5, 5)
    beacons = (
        _beacon("b1",  5.0,  3.0, "aa:bb:cc:dd:ee:01"),  # origin corner
        _beacon("b2", 25.0,  3.0, "aa:bb:cc:dd:ee:02"),  # 20 m east
        _beacon("b3", 15.0, 23.0, "aa:bb:cc:dd:ee:03"),  # 10 m east, 20 m north
    )
    # True device is at reading-frame (5, 5) = absolute (10, 8)
    true_rx, true_ry = 5.0, 5.0
    true_ax = true_rx + config.x_origin_m
    true_ay = true_ry + config.y_origin_m
    snapshot = []
    for b in beacons:
        d = math.sqrt(
            (b.x_m - true_ax) ** 2
            + (b.y_m - true_ay) ** 2
            + (b.z_m - config.device_z_m) ** 2
        )
        rssi = b.tx_power_dbm - 10.0 * b.path_loss_n * math.log10(max(d, 0.01))
        snapshot.append(_reading(b.mac_address, rssi))

    result = trilaterate_ble(snapshot, beacons, config, device_z_m=config.device_z_m)
    assert result is not None
    x, y = result
    assert abs(x - true_rx) < 0.5, f"x error {abs(x - true_rx):.2f} m"
    assert abs(y - true_ry) < 0.5, f"y error {abs(y - true_ry):.2f} m"
