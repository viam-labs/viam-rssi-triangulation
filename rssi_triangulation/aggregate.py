"""Combine multiple WiFi scan passes into averaged RSSI per BSSID."""

from __future__ import annotations

import statistics
import time
from collections import defaultdict
from dataclasses import dataclass

from .linux_scan import WifiReading, scan_wifi


@dataclass(frozen=True)
class AggregatedWifiReading:
    bssid: str
    ssid: str | None
    rssi_dbm: float
    rssi_std_dbm: float | None
    sample_count: int
    frequency_mhz: int | None
    backend: str


def aggregate_wifi_readings(
    batches: list[list[WifiReading]],
) -> list[AggregatedWifiReading]:
    """Aggregate RSSI per BSSID across scan batches (median when >=3 samples)."""
    by_bssid: dict[str, list[WifiReading]] = defaultdict(list)
    for batch in batches:
        for reading in batch:
            by_bssid[reading.bssid].append(reading)

    aggregated: list[AggregatedWifiReading] = []
    for bssid, samples in by_bssid.items():
        rssi_values = [s.rssi_dbm for s in samples]
        if len(rssi_values) >= 3:
            rssi_agg = statistics.median(rssi_values)
        else:
            rssi_agg = statistics.mean(rssi_values)
        rssi_std = statistics.stdev(rssi_values) if len(rssi_values) > 1 else None
        freqs = [s.frequency_mhz for s in samples if s.frequency_mhz is not None]
        freq = int(statistics.median(freqs)) if freqs else samples[0].frequency_mhz
        aggregated.append(
            AggregatedWifiReading(
                bssid=bssid,
                ssid=samples[0].ssid,
                rssi_dbm=rssi_agg,
                rssi_std_dbm=rssi_std,
                sample_count=len(samples),
                frequency_mhz=freq,
                backend=samples[0].backend,
            )
        )
    return aggregated


def collect_averaged_readings(
    *,
    scan_count: int,
    scan_delay_s: float,
    interface: str | None,
    network: str | None,
    backend: str | None,
    blocking: bool = False,
    fast_scan: bool = True,
) -> tuple[list[AggregatedWifiReading], str, int]:
    """
    Run `scan_count` WiFi scans and return RSSI averaged per BSSID.

    Returns (aggregated_readings, backend_name, scans_completed).
    """
    if scan_count < 1:
        raise ValueError("scan_count must be >= 1")

    batches: list[list[WifiReading]] = []
    backend_used = ""
    for i in range(scan_count):
        readings, backend_used = scan_wifi(
            interface=interface,
            network=network,
            backend=backend,
            blocking=blocking,
            fast=fast_scan,
        )
        batches.append(readings)
        delay = 0.0 if fast_scan else scan_delay_s
        if i + 1 < scan_count and delay > 0:
            time.sleep(delay)

    return aggregate_wifi_readings(batches), backend_used, scan_count
