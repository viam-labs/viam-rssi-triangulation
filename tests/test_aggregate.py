from __future__ import annotations

from rssi_triangulation.aggregate import aggregate_wifi_readings
from rssi_triangulation.linux_scan import WifiReading


def _reading(bssid: str, rssi: float) -> WifiReading:
    return WifiReading(
        bssid=bssid,
        ssid="Viam-5G",
        rssi_dbm=rssi,
        frequency_mhz=5180,
        backend="iw",
    )


def test_aggregate_mean_for_two_scans() -> None:
    batches = [
        [_reading("aa:bb:cc:dd:ee:01", -60.0)],
        [_reading("aa:bb:cc:dd:ee:01", -70.0)],
    ]
    out = aggregate_wifi_readings(batches)
    assert len(out) == 1
    assert out[0].bssid == "aa:bb:cc:dd:ee:01"
    assert out[0].rssi_dbm == -65.0
    assert out[0].sample_count == 2
    assert out[0].rssi_std_dbm is not None


def test_aggregate_median_for_three_or_more_samples() -> None:
    batches = [
        [_reading("aa:bb:cc:dd:ee:01", -50.0)],
        [_reading("aa:bb:cc:dd:ee:01", -70.0)],
        [_reading("aa:bb:cc:dd:ee:01", -60.0)],
    ]
    out = aggregate_wifi_readings(batches)
    assert out[0].rssi_dbm == -60.0


def test_aggregate_multiple_bssids() -> None:
    batches = [
        [
            _reading("aa:bb:cc:dd:ee:01", -60.0),
            _reading("aa:bb:cc:dd:ee:02", -80.0),
        ],
    ]
    out = aggregate_wifi_readings(batches)
    assert len(out) == 2
