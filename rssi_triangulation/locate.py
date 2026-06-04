"""Scan WiFi, match configured APs, and estimate floor position."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .aggregate import AggregatedWifiReading, collect_averaged_readings
from .aps import normalize_mac, resolve_ap_name
from .module_config import LocatorConfig, registry_from_config
from .registry import ApRegistry
from .fingerprint import FingerprintMatch, FingerprintStore, matched_to_rssi_dict
from .triangulate import (
    PositionEstimate,
    anchors_from_readings,
    estimate_position,
    filter_anchors,
)


@dataclass(frozen=True)
class PositionReading:
    x_m: float
    y_m: float

    def as_dict(self) -> dict[str, float]:
        return {"position_x_m": self.x_m, "position_y_m": self.y_m}


def bssid_lookup_from_registry(
    registry: ApRegistry,
    *,
    unifi_variants: bool = False,
) -> dict[str, str]:
    lookup = dict(registry.bssid_to_name)
    if not unifi_variants:
        return lookup
    from .aps import unifi_bssid_variants

    for ap in registry.access_points:
        for mac in unifi_bssid_variants(ap.bssid):
            lookup[mac] = ap.ap_name
    return lookup


def match_readings_to_aps(
    readings: list[AggregatedWifiReading],
    registry: ApRegistry,
    *,
    strict_mac: bool = True,
    min_sample_count: int = 1,
) -> list[tuple[str, float, float | None]]:
    """Return (ap_name, rssi_dbm, frequency_mhz) for known APs, strongest per name."""
    lookup = bssid_lookup_from_registry(registry, unifi_variants=not strict_mac)
    best: dict[str, tuple[str, float, float | None]] = {}

    for reading in readings:
        if reading.sample_count < min_sample_count:
            continue
        name = resolve_ap_name(reading.bssid, lookup)
        if name is None:
            continue
        prev = best.get(name)
        if prev is None or reading.rssi_dbm > prev[1]:
            best[name] = (name, reading.rssi_dbm, reading.frequency_mhz)

    return sorted(best.values(), key=lambda t: t[1], reverse=True)


def apply_floor_origin(
    estimate: PositionEstimate,
    *,
    x_origin_m: float,
    y_origin_m: float,
) -> PositionReading:
    return PositionReading(
        x_m=estimate.x_m - x_origin_m,
        y_m=estimate.y_m - y_origin_m,
    )


def smooth_position(
    previous: PositionReading | None,
    current: PositionReading,
    *,
    alpha: float = 0.25,
    max_step_m: float | None = 1.0,
) -> PositionReading:
    """Low-pass filter position and optionally limit one-reading jumps."""
    if previous is None:
        return current
    if alpha >= 1.0 and max_step_m is None:
        return current
    if alpha <= 0.0:
        return previous

    dx = current.x_m - previous.x_m
    dy = current.y_m - previous.y_m
    step_x = dx * alpha
    step_y = dy * alpha

    if max_step_m is not None and max_step_m > 0:
        step = math.hypot(step_x, step_y)
        if step > max_step_m:
            scale = max_step_m / step
            step_x *= scale
            step_y *= scale

    return PositionReading(
        x_m=previous.x_m + step_x,
        y_m=previous.y_m + step_y,
    )


def collect_matched_scan(
    config: LocatorConfig,
    *,
    interface: str | None = None,
    backend: str | None = None,
    scan_delay_s: float = 0.15,
    blocking: bool = False,
    strict_mac: bool = True,
    min_samples_per_ap: int | None = None,
    scan_count_override: int | None = None,
    fast_scan: bool = True,
) -> tuple[list[tuple[str, float, float | None]], str, list[AggregatedWifiReading], int]:
    """Run WiFi scans and return matched AP readings plus scan metadata."""
    registry = registry_from_config(config)
    scan_count = scan_count_override if scan_count_override is not None else config.scan_count
    aggregated, backend, scans_done = collect_averaged_readings(
        scan_count=scan_count,
        scan_delay_s=scan_delay_s,
        interface=interface,
        network=config.scan_ssid,
        backend=backend,
        blocking=blocking,
        fast_scan=fast_scan,
    )
    min_samples = min_samples_per_ap
    if min_samples is None:
        min_samples = 2 if scans_done >= 3 else 1
    matched = match_readings_to_aps(
        aggregated,
        registry,
        strict_mac=strict_mac,
        min_sample_count=min_samples,
    )
    return matched, backend, aggregated, scans_done


def estimate_fingerprint_position(
    store: FingerprintStore,
    matched: list[tuple[str, float, float | None]],
    *,
    k: int = 1,
    min_common_aps: int = 3,
    min_common_fraction: float = 0.5,
    max_rms_db: float | None = 10.0,
) -> FingerprintMatch | None:
    return store.match(
        matched_to_rssi_dict(matched),
        k=k,
        min_common_aps=min_common_aps,
        min_common_fraction=min_common_fraction,
        max_rms_db=max_rms_db,
    )


def fingerprint_confidence(
    match: FingerprintMatch,
    *,
    max_rms_db: float | None = 10.0,
    min_common_aps: int = 3,
) -> float:
    """
    How much to trust a fingerprint match when blending (0 = ignore, 1 = full weight).

    Tighter RMS and more overlapping APs increase confidence.
    """
    if not math.isfinite(match.distance_db):
        return 0.0
    if max_rms_db is not None and max_rms_db > 0 and match.distance_db >= max_rms_db:
        return 0.0
    if max_rms_db is not None and max_rms_db > 0:
        rms_factor = 1.0 - (match.distance_db / max_rms_db) ** 2
    else:
        rms_factor = 1.0 / (1.0 + match.distance_db)
    ap_factor = min(1.0, match.common_aps / max(min_common_aps, 1))
    return max(0.0, min(1.0, rms_factor * ap_factor))


def blend_fingerprint_with_centroid(
    centroid: PositionReading,
    fp_match: FingerprintMatch,
    *,
    max_blend: float = 0.5,
    max_rms_db: float | None = 10.0,
    min_common_aps: int = 3,
) -> tuple[PositionReading, FingerprintMatch]:
    """
    Blend geometric centroid with fingerprint k-NN in reading-frame coordinates.

    ``max_blend`` caps fingerprint influence (default 0.5 → at most half the correction).
    """
    cap = max(0.0, min(1.0, max_blend))
    weight = cap * fingerprint_confidence(
        fp_match,
        max_rms_db=max_rms_db,
        min_common_aps=min_common_aps,
    )
    blended = PositionReading(
        x_m=centroid.x_m * (1.0 - weight) + fp_match.x_m * weight,
        y_m=centroid.y_m * (1.0 - weight) + fp_match.y_m * weight,
    )
    annotated = FingerprintMatch(
        x_m=fp_match.x_m,
        y_m=fp_match.y_m,
        label=fp_match.label,
        distance_db=fp_match.distance_db,
        common_aps=fp_match.common_aps,
        k=fp_match.k,
        neighbors=fp_match.neighbors,
        blend_weight=weight,
    )
    return blended, annotated


def locate_position(
    config: LocatorConfig,
    *,
    interface: str | None = None,
    backend: str | None = None,
    scan_delay_s: float = 0.15,
    blocking: bool = False,
    strict_mac: bool = True,
    method: str = "hybrid",
    min_anchors: int = 3,
    max_rssi_delta_db: float | None = 35.0,
    min_rssi_dbm: float = -90.0,
    min_samples_per_ap: int | None = None,
    tx_power_dbm: float = -40.0,
    path_loss_n: float = 2.5,
    weight_temperature: float = 2.0,
    fingerprint_store: FingerprintStore | None = None,
    fingerprint_k: int = 1,
    fingerprint_min_common_aps: int = 3,
    fingerprint_min_common_fraction: float = 0.5,
    fingerprint_max_rms_db: float | None = 10.0,
    fingerprint_max_blend: float = 0.5,
    fingerprint_fallback: bool = True,
    fast_scan: bool = True,
) -> tuple[
    PositionReading,
    str,
    list[AggregatedWifiReading],
    int,
    str,
    FingerprintMatch | None,
]:
    """
    Run WiFi scans and estimate position in the configured coordinate frame.

    Returns (position, backend_name, aggregated_scan_readings, scans_completed, method_used).
    """
    registry = registry_from_config(config)
    matched, backend, aggregated, scans_done = collect_matched_scan(
        config,
        interface=interface,
        backend=backend,
        scan_delay_s=scan_delay_s,
        blocking=blocking,
        strict_mac=strict_mac,
        min_samples_per_ap=min_samples_per_ap,
        fast_scan=fast_scan,
    )

    method_used = method
    fp_match: FingerprintMatch | None = None
    has_db = False
    triangulation_method = (
        "weighted_centroid" if method in ("fingerprint", "hybrid") else method
    )

    if method in ("fingerprint", "hybrid"):
        has_db = fingerprint_store is not None and fingerprint_store.count() > 0
        if not has_db:
            if method == "hybrid" or fingerprint_fallback:
                method_used = "weighted_centroid"
            else:
                raise RuntimeError(
                    "fingerprint method requires a non-empty fingerprint database"
                )
        else:
            fp_match = estimate_fingerprint_position(
                fingerprint_store,
                matched,
                k=fingerprint_k,
                min_common_aps=fingerprint_min_common_aps,
                min_common_fraction=fingerprint_min_common_fraction,
                max_rms_db=None if method == "hybrid" else fingerprint_max_rms_db,
            )

    estimate = estimate_position(
        registry,
        matched,
        method=triangulation_method,
        min_anchors=min_anchors,
        tx_power_dbm=tx_power_dbm,
        path_loss_n=path_loss_n,
        max_rssi_delta_db=max_rssi_delta_db,
        min_rssi_dbm=min_rssi_dbm,
        weight_temperature=weight_temperature,
    )

    centroid_position: PositionReading | None = None
    if estimate is not None:
        centroid_position = apply_floor_origin(
            estimate,
            x_origin_m=config.x_origin_m,
            y_origin_m=config.y_origin_m,
        )

    if method == "hybrid" and has_db:
        if centroid_position is None and fp_match is None:
            raise RuntimeError(
                "hybrid method could not estimate: no centroid and no fingerprint match"
            )
        if centroid_position is None and fp_match is not None:
            return (
                PositionReading(x_m=fp_match.x_m, y_m=fp_match.y_m),
                backend,
                aggregated,
                scans_done,
                "fingerprint",
                fp_match,
            )
        if centroid_position is not None and fp_match is not None:
            blended, fp_match = blend_fingerprint_with_centroid(
                centroid_position,
                fp_match,
                max_blend=fingerprint_max_blend,
                max_rms_db=fingerprint_max_rms_db,
                min_common_aps=fingerprint_min_common_aps,
            )
            return blended, backend, aggregated, scans_done, "hybrid", fp_match
        if centroid_position is not None:
            return (
                centroid_position,
                backend,
                aggregated,
                scans_done,
                "weighted_centroid",
                None,
            )

    if method == "fingerprint" and fp_match is not None:
        return (
            PositionReading(x_m=fp_match.x_m, y_m=fp_match.y_m),
            backend,
            aggregated,
            scans_done,
            "fingerprint",
            fp_match,
        )
    if method == "fingerprint" and not fingerprint_fallback:
        raise RuntimeError(
            "no fingerprint match within max_rms_db / min_common_aps"
        )
    if method == "fingerprint":
        method_used = "weighted_centroid"

    if estimate is None:
        usable = filter_anchors(
            anchors_from_readings(registry, matched),
            max_delta_db=max_rssi_delta_db,
            min_rssi_dbm=min_rssi_dbm,
        )
        delta_desc = (
            "disabled"
            if max_rssi_delta_db is None
            else f"{max_rssi_delta_db:g} dB below the strongest"
        )
        raise RuntimeError(
            f"could not estimate position: matched {len(matched)} AP(s) by BSSID, "
            f"but only {len(usable)} passed the RSSI filters "
            f"(need ≥{min_anchors}). Most matched APs were weaker than "
            f"{min_rssi_dbm:g} dBm or more than {delta_desc}. Loosen min_rssi_dbm / "
            f"max_rssi_delta_db, or move closer to more configured APs."
        )
    assert centroid_position is not None
    return centroid_position, backend, aggregated, scans_done, method_used, fp_match
