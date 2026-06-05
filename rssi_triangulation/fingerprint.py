"""SQLite fingerprint store and k-NN RSSI matching."""

from __future__ import annotations

import json
import math
import sqlite3
import threading
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path


@dataclass(frozen=True)
class FingerprintRecord:
    id: int
    label: str
    x_m: float
    y_m: float
    z_m: float
    rssi_by_ap: dict[str, float]
    recorded_at: str
    scan_count: int


@dataclass(frozen=True)
class FingerprintMatch:
    x_m: float
    y_m: float
    z_m: float
    label: str
    distance_db: float
    common_aps: int
    k: int
    neighbors: tuple[str, ...]
    blend_weight: float = 0.0


def matched_to_rssi_dict(
    matched: list[tuple[str, float, float | None]],
) -> dict[str, float]:
    return {name: rssi for name, rssi, _freq in matched}


def normalize_rssi_vector(rssi_by_ap: dict[str, float]) -> dict[str, float]:
    """Relative RSSI (strongest AP = 0 dB) so matching uses shape, not absolute power."""
    if not rssi_by_ap:
        return {}
    peak = max(rssi_by_ap.values())
    return {name: rssi - peak for name, rssi in rssi_by_ap.items()}


def required_common_ap_count(
    a: dict[str, float],
    b: dict[str, float],
    *,
    min_common_aps: int,
    min_common_fraction: float,
) -> int:
    """Minimum overlapping APs required for a valid fingerprint comparison."""
    if not a or not b:
        return max(min_common_aps, 1)
    overlap = min(len(a), len(b))
    by_fraction = (
        math.ceil(overlap * min_common_fraction) if min_common_fraction > 0 else 0
    )
    return max(min_common_aps, by_fraction)


def rssi_vector_rms_db(
    a: dict[str, float],
    b: dict[str, float],
    *,
    normalize: bool = True,
    min_common_aps: int = 1,
    min_common_fraction: float = 0.0,
) -> tuple[float, int]:
    """
    RMS RSSI difference over AP names present in both vectors.

    When ``normalize`` is true (default), vectors are converted to relative RSSI
    before comparison so a match reflects which APs are stronger/weaker, not
    overall signal level (which varies scan-to-scan).
    """
    if normalize:
        a = normalize_rssi_vector(a)
        b = normalize_rssi_vector(b)
    common = set(a) & set(b)
    required = required_common_ap_count(
        a,
        b,
        min_common_aps=min_common_aps,
        min_common_fraction=min_common_fraction,
    )
    if len(common) < required:
        return float("inf"), len(common)
    err = sum((a[k] - b[k]) ** 2 for k in common) / len(common)
    return math.sqrt(err), len(common)


class FingerprintStore:
    """Thread-safe SQLite store for calibration fingerprints."""

    def __init__(self, path: Path | str) -> None:
        self._path = Path(path)
        self._lock = threading.Lock()
        self._cache: list[FingerprintRecord] | None = None
        self._cache_mtime: float | None = None
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._init_schema()

    def _invalidate_cache(self) -> None:
        self._cache = None
        self._cache_mtime = None

    def _db_mtime(self) -> float:
        if not self._path.exists():
            return 0.0
        return self._path.stat().st_mtime

    @property
    def path(self) -> Path:
        return self._path

    def _connect(self) -> sqlite3.Connection:
        conn = sqlite3.connect(self._path, timeout=30.0)
        conn.row_factory = sqlite3.Row
        return conn

    def _init_schema(self) -> None:
        with self._lock:
            with self._connect() as conn:
                conn.execute("PRAGMA journal_mode=WAL")
                conn.execute(
                    """
                    CREATE TABLE IF NOT EXISTS fingerprints (
                        id INTEGER PRIMARY KEY AUTOINCREMENT,
                        label TEXT NOT NULL UNIQUE,
                        x_m REAL NOT NULL,
                        y_m REAL NOT NULL,
                        z_m REAL NOT NULL DEFAULT 0,
                        rssi_json TEXT NOT NULL,
                        recorded_at TEXT NOT NULL,
                        scan_count INTEGER NOT NULL
                    )
                    """
                )
                cols = {
                    row[1]
                    for row in conn.execute("PRAGMA table_info(fingerprints)")
                }
                if "z_m" not in cols:
                    conn.execute(
                        "ALTER TABLE fingerprints ADD COLUMN z_m REAL NOT NULL DEFAULT 0"
                    )
                conn.commit()

    def record(
        self,
        label: str,
        *,
        x_m: float,
        y_m: float,
        z_m: float = 0.0,
        rssi_by_ap: dict[str, float],
        scan_count: int,
    ) -> FingerprintRecord:
        if not rssi_by_ap:
            raise ValueError("fingerprint has no AP RSSI readings")
        payload = json.dumps(rssi_by_ap, sort_keys=True)
        recorded_at = datetime.now(timezone.utc).isoformat()
        with self._lock:
            with self._connect() as conn:
                conn.execute(
                    """
                    INSERT INTO fingerprints (label, x_m, y_m, z_m, rssi_json, recorded_at, scan_count)
                    VALUES (?, ?, ?, ?, ?, ?, ?)
                    ON CONFLICT(label) DO UPDATE SET
                        x_m = excluded.x_m,
                        y_m = excluded.y_m,
                        z_m = excluded.z_m,
                        rssi_json = excluded.rssi_json,
                        recorded_at = excluded.recorded_at,
                        scan_count = excluded.scan_count
                    """,
                    (label, x_m, y_m, z_m, payload, recorded_at, scan_count),
                )
                row_id = conn.execute(
                    "SELECT id FROM fingerprints WHERE label = ?", (label,)
                ).fetchone()["id"]
                conn.commit()
            self._invalidate_cache()
        return FingerprintRecord(
            id=int(row_id),
            label=label,
            x_m=x_m,
            y_m=y_m,
            z_m=z_m,
            rssi_by_ap=rssi_by_ap,
            recorded_at=recorded_at,
            scan_count=scan_count,
        )

    def list_all(self) -> list[FingerprintRecord]:
        with self._lock:
            mtime = self._db_mtime()
            if self._cache is not None and self._cache_mtime == mtime:
                return list(self._cache)
            with self._connect() as conn:
                rows = conn.execute(
                    "SELECT id, label, x_m, y_m, z_m, rssi_json, recorded_at, scan_count "
                    "FROM fingerprints ORDER BY label"
                ).fetchall()
            self._cache = [_row_to_record(row) for row in rows]
            self._cache_mtime = mtime
            return list(self._cache)

    def count(self) -> int:
        with self._lock:
            if self._cache is not None and self._cache_mtime == self._db_mtime():
                return len(self._cache)
            with self._connect() as conn:
                row = conn.execute("SELECT COUNT(*) AS n FROM fingerprints").fetchone()
            return int(row["n"])

    def delete(self, label: str) -> bool:
        with self._lock:
            with self._connect() as conn:
                cur = conn.execute(
                    "DELETE FROM fingerprints WHERE label = ?", (label,)
                )
                conn.commit()
                deleted = cur.rowcount > 0
            if deleted:
                self._invalidate_cache()
            return deleted

    def clear(self) -> int:
        with self._lock:
            with self._connect() as conn:
                cur = conn.execute("DELETE FROM fingerprints")
                conn.commit()
                removed = cur.rowcount
            self._invalidate_cache()
            return removed

    def match(
        self,
        rssi_by_ap: dict[str, float],
        *,
        k: int = 1,
        min_common_aps: int = 3,
        min_common_fraction: float = 0.5,
        max_rms_db: float | None = 10.0,
        normalize: bool = True,
    ) -> FingerprintMatch | None:
        if not rssi_by_ap:
            return None
        k = max(1, k)
        candidates: list[tuple[float, int, FingerprintRecord]] = []
        for fp in self.list_all():
            dist, common = rssi_vector_rms_db(
                rssi_by_ap,
                fp.rssi_by_ap,
                normalize=normalize,
                min_common_aps=min_common_aps,
                min_common_fraction=min_common_fraction,
            )
            if not math.isfinite(dist):
                continue
            if max_rms_db is not None and dist > max_rms_db:
                continue
            candidates.append((dist, common, fp))

        if not candidates:
            return None

        candidates.sort(key=lambda t: t[0])
        top = candidates[:k]
        weights = [1.0 / (d + 0.1) for d, _, _ in top]
        wsum = sum(weights)
        x = sum(w * fp.x_m for (_, _, fp), w in zip(top, weights)) / wsum
        y = sum(w * fp.y_m for (_, _, fp), w in zip(top, weights)) / wsum
        z = sum(w * fp.z_m for (_, _, fp), w in zip(top, weights)) / wsum
        best_dist, best_common, best_fp = top[0]
        return FingerprintMatch(
            x_m=x,
            y_m=y,
            z_m=z,
            label=best_fp.label,
            distance_db=best_dist,
            common_aps=best_common,
            k=len(top),
            neighbors=tuple(fp.label for _, _, fp in top),
        )


def _row_to_record(row: sqlite3.Row) -> FingerprintRecord:
    keys = row.keys()
    z_m = float(row["z_m"]) if "z_m" in keys else 0.0
    return FingerprintRecord(
        id=int(row["id"]),
        label=str(row["label"]),
        x_m=float(row["x_m"]),
        y_m=float(row["y_m"]),
        z_m=z_m,
        rssi_by_ap=json.loads(row["rssi_json"]),
        recorded_at=str(row["recorded_at"]),
        scan_count=int(row["scan_count"]),
    )
