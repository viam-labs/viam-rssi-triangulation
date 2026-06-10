"""Fit path-loss parameters (tx_power_dbm, path_loss_n) from fingerprints.

Each stored fingerprint is an RSSI vector recorded at a known floor position,
so every (fingerprint, AP) pair is one observation of RSSI at a known
distance. Fitting the log-distance model

    rssi = tx_power_dbm - 10 * path_loss_n * log10(d)

is then ordinary least squares of RSSI against log10(distance).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .fingerprint import FingerprintRecord, FingerprintStore
from .module_config import LocatorConfig


@dataclass(frozen=True)
class PathLossSample:
    fingerprint_label: str
    ap_name: str
    distance_m: float
    rssi_dbm: float


@dataclass(frozen=True)
class PathLossFit:
    tx_power_dbm: float
    path_loss_n: float
    rmse_db: float
    sample_count: int
    fingerprint_count: int


# Below ~0.5 m log10(d) blows up and antenna near-field effects dominate.
MIN_SAMPLE_DISTANCE_M = 0.5

# Fit needs distance diversity: max/min distance ratio of at least ~2x.
MIN_LOG_DISTANCE_SPREAD = 0.3

MIN_SAMPLES = 4

# Sanity bounds; values outside these usually mean bad AP coordinates or
# too few / clustered fingerprints rather than a real propagation estimate.
PLAUSIBLE_N = (1.5, 6.0)
PLAUSIBLE_TX = (-70.0, -10.0)


def samples_from_fingerprints(
    config: LocatorConfig,
    records: list[FingerprintRecord],
    *,
    min_distance_m: float = MIN_SAMPLE_DISTANCE_M,
) -> list[PathLossSample]:
    """One sample per (fingerprint, known AP) pair, in reading-frame coords."""
    ap_positions = {
        ap.name: (
            ap.x_m - config.x_origin_m,
            ap.y_m - config.y_origin_m,
            ap.z_m,
        )
        for ap in config.access_points
    }
    samples: list[PathLossSample] = []
    for record in records:
        for ap_name, rssi in record.rssi_by_ap.items():
            pos = ap_positions.get(ap_name)
            if pos is None:
                continue
            distance = math.sqrt(
                (record.x_m - pos[0]) ** 2
                + (record.y_m - pos[1]) ** 2
                + (record.z_m - pos[2]) ** 2
            )
            if distance < min_distance_m:
                continue
            samples.append(
                PathLossSample(
                    fingerprint_label=record.label,
                    ap_name=ap_name,
                    distance_m=distance,
                    rssi_dbm=rssi,
                )
            )
    return samples


def path_loss_rmse_db(
    samples: list[PathLossSample],
    *,
    tx_power_dbm: float,
    path_loss_n: float,
) -> float:
    """RMS error in dB of the model against the samples."""
    if not samples:
        return 0.0
    err_sq = 0.0
    for s in samples:
        predicted = tx_power_dbm - 10.0 * path_loss_n * math.log10(s.distance_m)
        err_sq += (s.rssi_dbm - predicted) ** 2
    return math.sqrt(err_sq / len(samples))


def fit_path_loss(samples: list[PathLossSample]) -> PathLossFit:
    """Least-squares fit of rssi = tx - 10*n*log10(d) over the samples."""
    if len(samples) < MIN_SAMPLES:
        raise ValueError(
            f"need at least {MIN_SAMPLES} (fingerprint, AP) samples to calibrate, "
            f"got {len(samples)}. Record more fingerprints at known positions."
        )
    us = [math.log10(s.distance_m) for s in samples]
    rs = [s.rssi_dbm for s in samples]
    spread = max(us) - min(us)
    if spread < MIN_LOG_DISTANCE_SPREAD:
        raise ValueError(
            "fingerprint-to-AP distances are too similar to fit path loss "
            f"(distance ratio {10 ** spread:.2f}x, need >= "
            f"{10 ** MIN_LOG_DISTANCE_SPREAD:.1f}x). Record fingerprints both "
            "near and far from APs."
        )
    n = len(samples)
    mean_u = sum(us) / n
    mean_r = sum(rs) / n
    cov = sum((u - mean_u) * (r - mean_r) for u, r in zip(us, rs))
    var = sum((u - mean_u) ** 2 for u in us)
    slope = cov / var
    tx_power_dbm = mean_r - slope * mean_u
    path_loss_n = -slope / 10.0
    rmse = path_loss_rmse_db(
        samples, tx_power_dbm=tx_power_dbm, path_loss_n=path_loss_n
    )
    labels = {s.fingerprint_label for s in samples}
    return PathLossFit(
        tx_power_dbm=tx_power_dbm,
        path_loss_n=path_loss_n,
        rmse_db=rmse,
        sample_count=n,
        fingerprint_count=len(labels),
    )


def per_ap_residuals(
    samples: list[PathLossSample],
    *,
    tx_power_dbm: float,
    path_loss_n: float,
) -> list[dict]:
    """Mean residual (measured - predicted, dB) per AP, worst first.

    A single AP with a large |mean residual| while the rest fit well usually
    means that AP's configured coordinates (or BSSID mapping) are wrong.
    """
    by_ap: dict[str, list[float]] = {}
    for s in samples:
        predicted = tx_power_dbm - 10.0 * path_loss_n * math.log10(s.distance_m)
        by_ap.setdefault(s.ap_name, []).append(s.rssi_dbm - predicted)
    rows = [
        {
            "ap_name": name,
            "mean_residual_db": sum(errs) / len(errs),
            "sample_count": len(errs),
        }
        for name, errs in by_ap.items()
    ]
    rows.sort(key=lambda r: abs(r["mean_residual_db"]), reverse=True)
    return rows


def fit_warnings(fit: PathLossFit) -> list[str]:
    warnings: list[str] = []
    lo_n, hi_n = PLAUSIBLE_N
    if not lo_n <= fit.path_loss_n <= hi_n:
        warnings.append(
            f"fitted path_loss_n={fit.path_loss_n:.2f} is outside the plausible "
            f"indoor range [{lo_n}, {hi_n}]; check AP coordinates and fingerprint positions"
        )
    lo_tx, hi_tx = PLAUSIBLE_TX
    if not lo_tx <= fit.tx_power_dbm <= hi_tx:
        warnings.append(
            f"fitted tx_power_dbm={fit.tx_power_dbm:.1f} is outside the plausible "
            f"range [{lo_tx}, {hi_tx}]; check AP coordinates and fingerprint positions"
        )
    if fit.rmse_db > 8.0:
        warnings.append(
            f"fit rmse {fit.rmse_db:.1f} dB is high; the log-distance model only "
            "loosely matches this environment (walls/multipath), so expect "
            "coarse geometric accuracy and rely on fingerprints where possible"
        )
    return warnings


def calibrate_from_fingerprints(
    config: LocatorConfig,
    db: FingerprintStore,
    *,
    current_tx_power_dbm: float | None = None,
    current_path_loss_n: float | None = None,
) -> dict:
    """Fit path-loss parameters from all stored fingerprints.

    Returns a JSON-friendly result dict (used by do_command and the CLI).
    """
    records = db.list_all()
    samples = samples_from_fingerprints(config, records)
    fit = fit_path_loss(samples)
    result: dict = {
        "ok": True,
        "tx_power_dbm": round(fit.tx_power_dbm, 2),
        "path_loss_n": round(fit.path_loss_n, 3),
        "rmse_db": round(fit.rmse_db, 2),
        "sample_count": fit.sample_count,
        "fingerprint_count": fit.fingerprint_count,
        "per_ap_residuals_db": [
            {
                "ap_name": r["ap_name"],
                "mean_residual_db": round(r["mean_residual_db"], 2),
                "sample_count": r["sample_count"],
            }
            for r in per_ap_residuals(
                samples,
                tx_power_dbm=fit.tx_power_dbm,
                path_loss_n=fit.path_loss_n,
            )
        ],
        "db_path": str(db.path),
    }
    warnings = fit_warnings(fit)
    if warnings:
        result["warnings"] = warnings
    if current_tx_power_dbm is not None and current_path_loss_n is not None:
        result["current"] = {
            "tx_power_dbm": current_tx_power_dbm,
            "path_loss_n": current_path_loss_n,
            "rmse_db": round(
                path_loss_rmse_db(
                    samples,
                    tx_power_dbm=current_tx_power_dbm,
                    path_loss_n=current_path_loss_n,
                ),
                2,
            ),
        }
    return result
