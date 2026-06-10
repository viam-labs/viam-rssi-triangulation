"""Continuous background WiFi scanning with time-decayed RSSI aggregation.

WiFi scans on a single radio are serialized by the kernel, so they cannot run
in parallel — but they can be pipelined. ``BackgroundScanner`` keeps a daemon
thread scanning into a rolling, timestamped sample buffer; position requests
then aggregate the freshest window in milliseconds instead of blocking for
several seconds of scan passes.

Aggregation weights samples by recency (exponential decay), so while the robot
moves the estimate reflects where it *is*, not the average of where it has
been. While stationary, the window still smooths RSSI noise per AP in dB-space
before triangulation — which steadies the position without adding lag the way
position-space smoothing does.
"""

from __future__ import annotations

import math
import threading
import time
from collections import defaultdict, deque
from dataclasses import dataclass

from .aggregate import AggregatedWifiReading
from .linux_scan import WifiReading, scan_wifi


@dataclass(frozen=True)
class TimedSample:
    reading: WifiReading
    monotonic_s: float
    scan_seq: int


def decayed_aggregate(
    samples: list[TimedSample],
    *,
    now_s: float,
    window_s: float = 8.0,
    half_life_s: float = 2.5,
) -> list[AggregatedWifiReading]:
    """Aggregate timestamped samples per BSSID, weighting newer samples higher.

    Each sample's weight is ``0.5 ** (age / half_life_s)``; samples older than
    ``window_s`` are ignored entirely.
    """
    by_bssid: dict[str, list[TimedSample]] = defaultdict(list)
    for s in samples:
        if now_s - s.monotonic_s <= window_s:
            by_bssid[s.reading.bssid].append(s)

    half_life = max(half_life_s, 1e-3)
    aggregated: list[AggregatedWifiReading] = []
    for bssid, group in by_bssid.items():
        weights = [0.5 ** ((now_s - s.monotonic_s) / half_life) for s in group]
        wsum = sum(weights)
        mean = sum(w * s.reading.rssi_dbm for s, w in zip(group, weights)) / wsum
        if len(group) > 1:
            var = (
                sum(
                    w * (s.reading.rssi_dbm - mean) ** 2
                    for s, w in zip(group, weights)
                )
                / wsum
            )
            std: float | None = math.sqrt(var)
        else:
            std = None
        newest = max(group, key=lambda s: s.monotonic_s)
        freq = newest.reading.frequency_mhz
        if freq is None:
            freq = next(
                (
                    s.reading.frequency_mhz
                    for s in sorted(group, key=lambda s: -s.monotonic_s)
                    if s.reading.frequency_mhz is not None
                ),
                None,
            )
        aggregated.append(
            AggregatedWifiReading(
                bssid=bssid,
                ssid=newest.reading.ssid,
                rssi_dbm=mean,
                rssi_std_dbm=std,
                sample_count=len(group),
                frequency_mhz=freq,
                backend=newest.reading.backend,
            )
        )
    return aggregated


class RssiSampleBuffer:
    """Thread-safe rolling buffer of timestamped per-BSSID RSSI samples."""

    def __init__(self, window_s: float = 8.0) -> None:
        self.window_s = max(window_s, 0.5)
        self._samples: deque[TimedSample] = deque()
        self._lock = threading.Lock()
        self._seq = 0

    def add_batch(
        self,
        readings: list[WifiReading],
        *,
        monotonic_s: float | None = None,
    ) -> int:
        """Append one scan pass; returns the scan sequence number."""
        now = time.monotonic() if monotonic_s is None else monotonic_s
        with self._lock:
            self._seq += 1
            seq = self._seq
            for r in readings:
                self._samples.append(
                    TimedSample(reading=r, monotonic_s=now, scan_seq=seq)
                )
            # Keep a little slack beyond the window so a slightly late
            # snapshot still sees a full window.
            cutoff = now - self.window_s * 1.5
            while self._samples and self._samples[0].monotonic_s < cutoff:
                self._samples.popleft()
        return seq

    def snapshot(
        self,
        *,
        now_s: float | None = None,
        half_life_s: float = 2.5,
    ) -> tuple[list[AggregatedWifiReading], int]:
        """Aggregated readings for the current window plus scan-pass count."""
        now = time.monotonic() if now_s is None else now_s
        with self._lock:
            samples = list(self._samples)
        in_window = [s for s in samples if now - s.monotonic_s <= self.window_s]
        scans = len({s.scan_seq for s in in_window})
        return (
            decayed_aggregate(
                in_window,
                now_s=now,
                window_s=self.window_s,
                half_life_s=half_life_s,
            ),
            scans,
        )


class BackgroundScanner:
    """Daemon thread that scans WiFi continuously into an ``RssiSampleBuffer``.

    Scan failures are recorded (``last_error``) and retried with a backoff
    rather than killing the thread, so a transiently busy interface recovers.
    """

    def __init__(
        self,
        *,
        interface: str | None = None,
        network: str | None = None,
        backend: str | None = None,
        fast: bool = True,
        blocking: bool = False,
        interval_s: float = 0.5,
        window_s: float = 8.0,
        half_life_s: float = 2.5,
    ) -> None:
        self._interface = interface
        self._network = network
        self._backend = backend
        self._fast = fast
        self._blocking = blocking
        self.interval_s = max(interval_s, 0.05)
        self.half_life_s = half_life_s
        self.buffer = RssiSampleBuffer(window_s)
        self._stop = threading.Event()
        self._first_scan = threading.Event()
        self._thread: threading.Thread | None = None
        self._backend_used = ""
        self._last_error: str | None = None

    @property
    def backend_used(self) -> str:
        return self._backend_used

    @property
    def last_error(self) -> str | None:
        return self._last_error

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> None:
        if self.running:
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, name="rssi-bg-scan", daemon=True
        )
        self._thread.start()

    def stop(self, timeout_s: float = 3.0) -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=timeout_s)
        self._thread = None

    def wait_for_data(self, timeout_s: float = 5.0) -> bool:
        """Block until the first scan pass lands (or timeout). True if data."""
        return self._first_scan.wait(timeout=timeout_s)

    def snapshot(self) -> tuple[list[AggregatedWifiReading], str, int]:
        """(aggregated_readings, backend_name, scan_passes_in_window)."""
        aggregated, scans = self.buffer.snapshot(half_life_s=self.half_life_s)
        return aggregated, self._backend_used or "background", scans

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                readings, backend = scan_wifi(
                    interface=self._interface,
                    network=self._network,
                    backend=self._backend,
                    blocking=self._blocking,
                    fast=self._fast,
                )
                self._backend_used = backend
                self._last_error = None
                self.buffer.add_batch(readings)
                self._first_scan.set()
                self._stop.wait(self.interval_s)
            except Exception as exc:  # keep scanning despite transient failures
                self._last_error = str(exc)
                self._stop.wait(max(self.interval_s, 1.0))
