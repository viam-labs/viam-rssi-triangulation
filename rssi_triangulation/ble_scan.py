"""Background BLE scanner for beacon-based position ranging.

Architecture mirrors ``BackgroundScanner`` (WiFi): a daemon thread owns a
dedicated ``asyncio`` event loop and runs ``bleak.BleakScanner`` continuously,
storing advertisement readings in a thread-safe rolling buffer.  The main
thread calls ``snapshot()`` to get recent readings, then ``trilaterate_ble()``
converts them to a 2D position fix using the configured beacon positions.

``bleak`` is an optional dependency — an ``ImportError`` is raised only when
``BackgroundBleScanner`` is instantiated, not at import time.
"""

from __future__ import annotations

import asyncio
import collections
import math
import threading
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .module_config import BleBeacon, LocatorConfig


# ---------------------------------------------------------------------------
# Data types
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class BleReading:
    """A single BLE advertisement received from one device."""

    mac: str          # lowercase colon-separated MAC address
    rssi_dbm: float
    name: str         # advertised name, may be empty
    timestamp_s: float  # monotonic clock


# ---------------------------------------------------------------------------
# Thread-safe rolling buffer
# ---------------------------------------------------------------------------


class BleScanBuffer:
    """Thread-safe rolling buffer of ``BleReading`` records.

    Keeps only the most recent reading per MAC address up to ``maxlen``
    unique devices.  Readings older than ``max_age_s`` are dropped on
    ``snapshot()``.
    """

    def __init__(self, *, maxlen: int = 128, max_age_s: float = 5.0) -> None:
        self._maxlen = maxlen
        self._max_age_s = max_age_s
        self._lock = threading.Lock()
        # deque of BleReading; latest per MAC wins via dict on snapshot
        self._buf: collections.deque[BleReading] = collections.deque(maxlen=maxlen)

    def add(self, reading: BleReading) -> None:
        with self._lock:
            self._buf.append(reading)

    def snapshot(self) -> list[BleReading]:
        """Return the most recent reading per MAC, dropping stale entries."""
        now = time.monotonic()
        cutoff = now - self._max_age_s
        with self._lock:
            items = list(self._buf)
        # Keep newest reading per MAC within the age window
        by_mac: dict[str, BleReading] = {}
        for r in items:
            if r.timestamp_s < cutoff:
                continue
            existing = by_mac.get(r.mac)
            if existing is None or r.timestamp_s > existing.timestamp_s:
                by_mac[r.mac] = r
        return list(by_mac.values())

    def clear(self) -> None:
        with self._lock:
            self._buf.clear()


# ---------------------------------------------------------------------------
# Background scanner (daemon thread + bleak event loop)
# ---------------------------------------------------------------------------


class BackgroundBleScanner:
    """Daemon thread that continuously scans for BLE advertisements.

    Uses ``bleak.BleakScanner`` in passive scanning mode (callback-driven).
    The scanner runs inside a private ``asyncio`` event loop on a daemon
    thread so it does not block the Viam sensor loop.

    Usage::

        scanner = BackgroundBleScanner(scan_interval_s=1.0)
        scanner.start()
        ...
        readings = scanner.snapshot()  # list[BleReading]
        ...
        scanner.stop()

    Raises ``ImportError`` if ``bleak`` is not installed.
    """

    def __init__(
        self,
        *,
        scan_interval_s: float = 1.0,
        buffer_max_age_s: float = 5.0,
        buffer_maxlen: int = 256,
    ) -> None:
        try:
            import bleak as _bleak  # noqa: F401  (validate presence)
        except ImportError as exc:
            raise ImportError(
                "bleak is required for BLE beacon support. "
                "Install it with: pip install bleak"
            ) from exc

        self._scan_interval_s = max(scan_interval_s, 0.1)
        self._buffer = BleScanBuffer(
            maxlen=buffer_maxlen, max_age_s=buffer_max_age_s
        )
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._started = threading.Event()
        self.last_error: str | None = None

    def start(self) -> None:
        """Start the background scanning thread."""
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run, name="ble-scanner", daemon=True
        )
        self._thread.start()
        self._started.wait(timeout=5.0)

    def stop(self) -> None:
        """Stop the background scanning thread and wait for it to exit."""
        self._stop_event.set()
        loop = self._loop
        if loop is not None and loop.is_running():
            loop.call_soon_threadsafe(loop.stop)
        if self._thread is not None:
            self._thread.join(timeout=5.0)
        self._thread = None
        self._loop = None

    def snapshot(self) -> list[BleReading]:
        """Return the most recent reading per visible BLE device."""
        return self._buffer.snapshot()

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _run(self) -> None:
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._scan_loop())
        finally:
            try:
                self._loop.close()
            except Exception:
                pass

    async def _scan_loop(self) -> None:
        from bleak import BleakScanner
        from bleak.backends.device import BLEDevice
        from bleak.backends.scanner import AdvertisementData

        def _callback(device: BLEDevice, adv: AdvertisementData) -> None:
            rssi = adv.rssi
            if rssi is None:
                return
            reading = BleReading(
                mac=device.address.lower().replace("-", ":"),
                rssi_dbm=float(rssi),
                name=device.name or "",
                timestamp_s=time.monotonic(),
            )
            self._buffer.add(reading)

        self._started.set()
        while not self._stop_event.is_set():
            try:
                async with BleakScanner(detection_callback=_callback):
                    # Let the scanner collect advertisements for one interval,
                    # then loop so we can check the stop event.
                    await asyncio.sleep(self._scan_interval_s)
            except asyncio.CancelledError:
                break
            except Exception as exc:
                self.last_error = str(exc)
                # Back off briefly to avoid a tight error loop.
                try:
                    await asyncio.sleep(1.0)
                except asyncio.CancelledError:
                    break


# ---------------------------------------------------------------------------
# RSSI → distance (path-loss model, same as WiFi)
# ---------------------------------------------------------------------------


def ble_rssi_to_distance_m(
    rssi_dbm: float,
    tx_power_dbm: float,
    path_loss_n: float,
) -> float:
    """Convert BLE RSSI to estimated distance via the log-distance path-loss model.

    ``d = 10 ^ ((TxPower − RSSI) / (10 · n))``

    Clamps to a minimum of 0.01 m to avoid zero/negative distances.
    """
    exp = (tx_power_dbm - rssi_dbm) / (10.0 * max(path_loss_n, 0.1))
    return max(10.0 ** exp, 0.01)


# ---------------------------------------------------------------------------
# Trilateration helper
# ---------------------------------------------------------------------------


def trilaterate_ble(
    snapshot: list[BleReading],
    beacons: tuple["BleBeacon", ...],
    config: "LocatorConfig",
    *,
    device_z_m: float = 0.0,
    min_rssi_dbm: float = -90.0,
    prior_x: float | None = None,
    prior_y: float | None = None,
) -> tuple[float, float] | None:
    """Compute a 2D position fix from a BLE snapshot.

    Matches readings to the configured ``beacons`` by MAC address, converts
    RSSI to range via each beacon's path-loss parameters, then calls the
    existing ``geo`` circle-intersection trilateration.

    Positions are returned in the **reading frame** (floor origin subtracted),
    consistent with the WiFi position output.

    Returns ``None`` when fewer than 1 beacon is visible or trilateration
    raises a geometry error.
    """
    from . import geo as _geo  # local import to avoid circular dependency

    # MAC → beacon lookup
    mac_to_beacon = {b.mac_address.lower(): b for b in beacons}

    # Build the ap_ranges dict expected by geo.infer_xy_from_slant_distances:
    # maps beacon name → (x_reading_frame, y_reading_frame, z, slant_m)
    ap_ranges: dict[str, tuple[float, float, float, float]] = {}
    for reading in snapshot:
        if reading.rssi_dbm < min_rssi_dbm:
            continue
        beacon = mac_to_beacon.get(reading.mac.lower())
        if beacon is None:
            continue
        distance_m = ble_rssi_to_distance_m(
            reading.rssi_dbm, beacon.tx_power_dbm, beacon.path_loss_n
        )
        # Translate from absolute floor coordinates to the reading frame
        bx = beacon.x_m - config.x_origin_m
        by = beacon.y_m - config.y_origin_m
        ap_ranges[beacon.name] = (bx, by, beacon.z_m, distance_m)

    if len(ap_ranges) == 0:
        return None

    if len(ap_ranges) >= 2:
        try:
            return _geo.infer_xy_from_slant_distances(
                ap_ranges, device_z_m=device_z_m, config=config
            )
        except (ValueError, ZeroDivisionError):
            pass

    # Single beacon: range only — project toward the prior position.
    if prior_x is not None and prior_y is not None:
        name, (bx, by, bz, dist) = next(iter(ap_ranges.items()))
        try:
            return _geo.infer_xy_from_single_slant_with_prior(
                bx, by, bz, dist,
                device_z_m=device_z_m,
                prior_x=prior_x,
                prior_y=prior_y,
                config=config,
            )
        except (ValueError, ZeroDivisionError):
            pass

    return None


def beacon_count_from_snapshot(
    snapshot: list[BleReading],
    beacons: tuple["BleBeacon", ...],
    *,
    min_rssi_dbm: float = -90.0,
) -> int:
    """Count the number of configured beacons visible above the RSSI floor."""
    mac_set = {b.mac_address.lower() for b in beacons}
    return sum(
        1 for r in snapshot
        if r.rssi_dbm >= min_rssi_dbm and r.mac.lower() in mac_set
    )
