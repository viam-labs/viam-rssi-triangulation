"""In-memory fingerprint recording sessions (start / sample / stop)."""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, Mapping


def aggregate_rssi_samples(
    samples: list[dict[str, float]],
) -> tuple[dict[str, float], dict[str, dict[str, float]]]:
    """
    Collapse repeated scans into per-AP mean RSSI and summary stats.

    ``rssi_stats_by_ap`` values: ``mean_dbm``, ``std_dbm``, ``n``.
    """
    if not samples:
        raise ValueError("no RSSI samples to aggregate")
    ap_names = sorted({ap for sample in samples for ap in sample})
    means: dict[str, float] = {}
    stats: dict[str, dict[str, float]] = {}
    for ap in ap_names:
        values = [sample[ap] for sample in samples if ap in sample]
        if not values:
            continue
        mean = sum(values) / len(values)
        means[ap] = mean
        if len(values) > 1:
            var = sum((v - mean) ** 2 for v in values) / len(values)
            std = math.sqrt(var)
        else:
            std = 0.0
        stats[ap] = {
            "mean_dbm": mean,
            "std_dbm": std,
            "n": float(len(values)),
        }
    if not means:
        raise ValueError("no overlapping AP RSSI across samples")
    return means, stats


@dataclass
class FingerprintRecordingSession:
    """Accumulates WiFi scans between start and stop."""

    started_at: str
    label: str | None = None
    x_m: float | None = None
    y_m: float | None = None
    z_m: float | None = None
    positioned: bool = False
    distances_by_ap: dict[str, float] = field(default_factory=dict)
    min_samples: int = 5
    auto_sample: bool = True
    samples: list[dict[str, float]] = field(default_factory=list)
    prior_xy_samples: list[tuple[float, float]] = field(default_factory=list)

    def add_sample(
        self,
        rssi_by_ap: dict[str, float],
        *,
        prior_xy: tuple[float, float] | None = None,
    ) -> None:
        if not rssi_by_ap:
            return
        self.samples.append(dict(rssi_by_ap))
        if prior_xy is not None:
            self.prior_xy_samples.append(prior_xy)

    def status(self) -> dict[str, Any]:
        return {
            "active": True,
            "label": self.label,
            "started_at": self.started_at,
            "sample_count": len(self.samples),
            "min_samples": self.min_samples,
            "auto_sample": self.auto_sample,
            "x_m": self.x_m,
            "y_m": self.y_m,
            "z_m": self.z_m,
            "positioned": self.positioned,
            "distances_m": dict(self.distances_by_ap),
        }


class FingerprintSessionManager:
    """Process-wide recording session (one active session at a time)."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._session: FingerprintRecordingSession | None = None

    def active(self) -> bool:
        with self._lock:
            return self._session is not None

    def status(self) -> dict[str, Any]:
        with self._lock:
            if self._session is None:
                return {"active": False}
            return self._session.status()

    def start(
        self,
        *,
        label: str | None = None,
        x_m: float | None = None,
        y_m: float | None = None,
        z_m: float | None = None,
        positioned: bool = False,
        distances_by_ap: Mapping[str, float] | None = None,
        min_samples: int = 5,
        auto_sample: bool = True,
    ) -> dict[str, Any]:
        if min_samples < 1:
            raise ValueError("min_samples must be >= 1")
        with self._lock:
            if self._session is not None:
                raise RuntimeError(
                    "fingerprint recording already active; stop or cancel it first"
                )
            self._session = FingerprintRecordingSession(
                started_at=datetime.now(timezone.utc).isoformat(),
                label=label,
                x_m=x_m,
                y_m=y_m,
                z_m=z_m,
                positioned=positioned,
                distances_by_ap=dict(distances_by_ap or {}),
                min_samples=min_samples,
                auto_sample=auto_sample,
            )
            return self._session.status()

    def cancel(self) -> dict[str, Any]:
        with self._lock:
            if self._session is None:
                return {"ok": True, "active": False, "discarded_samples": 0}
            discarded = len(self._session.samples)
            self._session = None
            return {
                "ok": True,
                "command": "cancel_fingerprint_recording",
                "discarded_samples": discarded,
            }

    def add_sample(
        self,
        rssi_by_ap: dict[str, float],
        *,
        prior_xy: tuple[float, float] | None = None,
    ) -> int:
        with self._lock:
            if self._session is None:
                return 0
            self._session.add_sample(rssi_by_ap, prior_xy=prior_xy)
            return len(self._session.samples)

    def ingest_matched(
        self,
        matched: list[tuple[str, float, float | None]],
        *,
        prior_xy: tuple[float, float] | None = None,
    ) -> int:
        from .fingerprint import matched_to_rssi_dict

        with self._lock:
            session = self._session
            if session is None or not session.auto_sample:
                return 0
        return self.add_sample(
            matched_to_rssi_dict(matched),
            prior_xy=prior_xy,
        )

    def stop(
        self,
        *,
        label: str | None = None,
        x_m: float | None = None,
        y_m: float | None = None,
        z_m: float | None = None,
        positioned: bool | None = None,
        distances_by_ap: Mapping[str, float] | None = None,
    ) -> tuple[FingerprintRecordingSession, dict[str, float], dict[str, dict[str, float]]]:
        with self._lock:
            session = self._session
            if session is None:
                raise RuntimeError("no active fingerprint recording session")
            if label is not None:
                session.label = label
            if x_m is not None:
                session.x_m = x_m
            if y_m is not None:
                session.y_m = y_m
            if z_m is not None:
                session.z_m = z_m
            if positioned is not None:
                session.positioned = positioned
            if distances_by_ap is not None:
                session.distances_by_ap = dict(distances_by_ap)
            if not session.label:
                raise ValueError(
                    'stop_fingerprint_recording requires "label" (at start or stop)'
                )
            if len(session.samples) < session.min_samples:
                raise RuntimeError(
                    f"fingerprint recording has {len(session.samples)} sample(s); "
                    f"need at least {session.min_samples}. Keep the device still, "
                    "call get_readings() / run scans while recording, or lower "
                    "min_samples."
                )
            means, stats = aggregate_rssi_samples(session.samples)
            finished = session
            self._session = None
            return finished, means, stats


_SESSION_MANAGER = FingerprintSessionManager()


def get_fingerprint_session_manager() -> FingerprintSessionManager:
    return _SESSION_MANAGER
