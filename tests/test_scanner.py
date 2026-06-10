from __future__ import annotations

import time

from rssi_triangulation.linux_scan import WifiReading
from rssi_triangulation.scanner import (
    BackgroundScanner,
    RssiSampleBuffer,
    TimedSample,
    decayed_aggregate,
)


def _reading(bssid: str = "aa:bb:cc:dd:ee:01", rssi: float = -60.0) -> WifiReading:
    return WifiReading(
        bssid=bssid,
        ssid="Net",
        rssi_dbm=rssi,
        frequency_mhz=5500,
        backend="test",
    )


def _sample(rssi: float, t: float, seq: int = 1, bssid: str = "aa:bb:cc:dd:ee:01") -> TimedSample:
    return TimedSample(reading=_reading(bssid, rssi), monotonic_s=t, scan_seq=seq)


def test_decayed_aggregate_weights_newer_samples_higher() -> None:
    # Old sample at -80, fresh sample at -50: result should sit near -50.
    samples = [_sample(-80.0, t=0.0, seq=1), _sample(-50.0, t=10.0, seq=2)]
    out = decayed_aggregate(samples, now_s=10.0, window_s=60.0, half_life_s=2.0)
    assert len(out) == 1
    assert out[0].rssi_dbm > -52.0
    assert out[0].sample_count == 2


def test_decayed_aggregate_drops_samples_outside_window() -> None:
    samples = [_sample(-80.0, t=0.0, seq=1), _sample(-50.0, t=10.0, seq=2)]
    out = decayed_aggregate(samples, now_s=10.0, window_s=5.0, half_life_s=2.0)
    assert len(out) == 1
    assert out[0].rssi_dbm == -50.0
    assert out[0].sample_count == 1


def test_decayed_aggregate_groups_by_bssid() -> None:
    samples = [
        _sample(-60.0, t=1.0, bssid="aa:bb:cc:dd:ee:01"),
        _sample(-70.0, t=1.0, bssid="aa:bb:cc:dd:ee:02"),
    ]
    out = decayed_aggregate(samples, now_s=1.0, window_s=5.0, half_life_s=2.0)
    assert {r.bssid for r in out} == {"aa:bb:cc:dd:ee:01", "aa:bb:cc:dd:ee:02"}


def test_buffer_snapshot_counts_scan_passes_in_window() -> None:
    buf = RssiSampleBuffer(window_s=10.0)
    buf.add_batch([_reading(rssi=-60.0)], monotonic_s=100.0)
    buf.add_batch([_reading(rssi=-62.0)], monotonic_s=101.0)
    buf.add_batch([_reading(rssi=-64.0)], monotonic_s=102.0)
    aggregated, scans = buf.snapshot(now_s=102.0)
    assert scans == 3
    assert len(aggregated) == 1
    assert aggregated[0].sample_count == 3


def test_buffer_prunes_old_samples() -> None:
    buf = RssiSampleBuffer(window_s=2.0)
    buf.add_batch([_reading(rssi=-60.0)], monotonic_s=0.0)
    buf.add_batch([_reading(rssi=-50.0)], monotonic_s=100.0)
    aggregated, scans = buf.snapshot(now_s=100.0)
    assert scans == 1
    assert aggregated[0].rssi_dbm == -50.0


def test_background_scanner_collects_and_stops(monkeypatch) -> None:
    calls: list[int] = []

    def fake_scan_wifi(**kwargs):
        calls.append(1)
        return [_reading(rssi=-55.0)], "fake"

    monkeypatch.setattr("rssi_triangulation.scanner.scan_wifi", fake_scan_wifi)
    scanner = BackgroundScanner(network="Net", interval_s=0.05, window_s=5.0)
    scanner.start()
    try:
        assert scanner.wait_for_data(timeout_s=2.0)
        aggregated, backend, scans = scanner.snapshot()
        assert backend == "fake"
        assert scans >= 1
        assert aggregated[0].bssid == "aa:bb:cc:dd:ee:01"
    finally:
        scanner.stop()
    assert not scanner.running
    assert calls


def test_background_scanner_survives_scan_errors(monkeypatch) -> None:
    state = {"n": 0}

    def flaky_scan_wifi(**kwargs):
        state["n"] += 1
        if state["n"] == 1:
            raise RuntimeError("device busy")
        return [_reading(rssi=-58.0)], "fake"

    monkeypatch.setattr("rssi_triangulation.scanner.scan_wifi", flaky_scan_wifi)
    scanner = BackgroundScanner(network="Net", interval_s=0.05, window_s=5.0)
    scanner.start()
    try:
        # First pass fails (1s backoff), second succeeds.
        assert scanner.wait_for_data(timeout_s=3.0)
        aggregated, _, _ = scanner.snapshot()
        assert aggregated
    finally:
        scanner.stop()
