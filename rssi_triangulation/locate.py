"""Scan WiFi, match configured APs, and estimate floor position."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .aggregate import AggregatedWifiReading, collect_averaged_readings
from .aps import normalize_mac, resolve_ap_name
from .calibrate import MIN_SAMPLE_DISTANCE_M
from .geo import (
    infer_xy_from_single_slant_with_prior,
    infer_xy_from_slant_distances,
    slant_range_m,
)
from .module_config import LocatorConfig, registry_from_config
from .registry import ApRegistry
from .fingerprint import (
    FingerprintMatch,
    FingerprintRecord,
    FingerprintStore,
    matched_to_rssi_dict,
)
from .triangulate import (
    PositionEstimate,
    anchors_from_readings,
    combined_anchor_weights,
    effective_anchors,
    estimate_position,
    filter_anchors,
)

# Minimum blend scale for uncertain range_prior matches (no laser trust without RSSI agreement).
RANGE_PRIOR_BLEND_FACTOR = 0.55


@dataclass(frozen=True)
class PositionReading:
    x_m: float
    y_m: float
    z_m: float = 0.0

    def as_dict(self) -> dict:
        return {
            "location": {
                "x": self.x_m,
                "y": self.y_m,
                "z": self.z_m,
                "unit": "meters",
            }
        }

    def with_z(self, z_m: float) -> PositionReading:
        return PositionReading(x_m=self.x_m, y_m=self.y_m, z_m=z_m)


def access_points_relative_to_position(
    position: PositionReading,
    matched: list[tuple[str, float, float | None]],
    config: LocatorConfig,
) -> list[dict]:
    """
    APs heard this scan, strongest RSSI first.

    x/y/z are offsets from the antenna position to each AP (AP − current), in
    meters. ``range_m`` is the 3D slant distance (compare to laser ``distance_to_ap``).
    """
    ap_by_name = {ap.name: ap for ap in config.access_points}
    rows: list[dict] = []
    for name, rssi_dbm, _freq in sorted(matched, key=lambda t: t[1], reverse=True):
        ap = ap_by_name.get(name)
        if ap is None:
            continue
        ap_x = ap.x_m - config.x_origin_m
        ap_y = ap.y_m - config.y_origin_m
        dx = ap_x - position.x_m
        dy = ap_y - position.y_m
        dz = ap.z_m - position.z_m
        rows.append(
            {
                "name": name,
                "x": dx,
                "y": dy,
                "z": dz,
                "range_m": slant_range_m(
                    position.x_m,
                    position.y_m,
                    device_z_m=position.z_m,
                    ap_x=ap_x,
                    ap_y=ap_y,
                    ap_z=ap.z_m,
                ),
                "unit": "meters",
                "bssid": ap.bssid,
                "rssi": rssi_dbm,
            }
        )
    return rows


def ap_position_in_reading_frame(
    config: LocatorConfig, ap_name: str
) -> tuple[float, float, float]:
    for ap in config.access_points:
        if ap.name == ap_name:
            return (
                ap.x_m - config.x_origin_m,
                ap.y_m - config.y_origin_m,
                ap.z_m,
            )
    names = ", ".join(a.name for a in config.access_points)
    raise ValueError(f"unknown ap_name {ap_name!r}; configured APs: {names}")


def effective_fingerprint_position(
    record: FingerprintRecord,
    config: LocatorConfig,
    *,
    device_z_m: float,
    prior_xy: tuple[float, float] | None,
) -> tuple[float, float, float, bool, str | None]:
    """
    Resolve a fingerprint's floor coordinates for blending or display.

    Stored x/y are used when present. Otherwise a single laser slant range
    fixes distance to one AP; bearing is taken from ``prior_xy`` (the live
    WiFi geometry estimate). Two or more ranges use trilateration when possible.
    """
    if record.positioned and (record.x_m != 0.0 or record.y_m != 0.0):
        z = record.z_m if record.z_m else device_z_m
        return record.x_m, record.y_m, z, True, "stored"

    distances = dict(record.distances_by_ap or {})
    if not distances:
        if record.positioned:
            return record.x_m, record.y_m, record.z_m, True, "stored"
        return 0.0, 0.0, device_z_m, False, None

    if len(distances) >= 2:
        try:
            ap_ranges = {
                ap_name: (*ap_position_in_reading_frame(config, ap_name), slant)
                for ap_name, slant in distances.items()
            }
            xy = infer_xy_from_slant_distances(
                ap_ranges,
                device_z_m=device_z_m,
                config=config,
                min_horizontal_m=MIN_SAMPLE_DISTANCE_M,
            )
            if xy is not None:
                return xy[0], xy[1], device_z_m, True, "trilateration"
        except ValueError:
            pass

    if len(distances) == 1 and prior_xy is not None:
        ap_name, slant = next(iter(distances.items()))
        try:
            ax, ay, az = ap_position_in_reading_frame(config, ap_name)
            xy = infer_xy_from_single_slant_with_prior(
                ax,
                ay,
                az,
                slant,
                device_z_m=device_z_m,
                prior_x=prior_xy[0],
                prior_y=prior_xy[1],
                config=config,
                min_horizontal_m=MIN_SAMPLE_DISTANCE_M,
            )
            return xy[0], xy[1], device_z_m, True, "range_prior"
        except ValueError:
            pass

    if record.positioned:
        return record.x_m, record.y_m, record.z_m, True, "stored"
    return 0.0, 0.0, device_z_m, False, None


def geometric_centroid_xy(
    config: LocatorConfig,
    matched: list[tuple[str, float, float | None]],
    *,
    device_z_m: float | None = None,
    min_anchors: int = 3,
    max_rssi_delta_db: float | None = 20.0,
    min_rssi_dbm: float = -82.0,
    tx_power_dbm: float = -40.0,
    path_loss_n: float = 2.5,
    weight_temperature: float = 2.0,
) -> tuple[float, float] | None:
    """Reading-frame (x, y) from WiFi geometry only, without fingerprint blending."""
    registry = registry_from_config(config)
    effective_device_z = (
        config.device_z_m if device_z_m is None else device_z_m
    )
    estimate = estimate_position(
        registry,
        matched,
        method="weighted_centroid",
        min_anchors=min_anchors,
        device_z_m=effective_device_z,
        tx_power_dbm=tx_power_dbm,
        path_loss_n=path_loss_n,
        max_rssi_delta_db=max_rssi_delta_db,
        min_rssi_dbm=min_rssi_dbm,
        weight_temperature=weight_temperature,
    )
    if estimate is None:
        return None
    pos = apply_floor_origin(
        estimate,
        x_origin_m=config.x_origin_m,
        y_origin_m=config.y_origin_m,
    )
    return pos.x_m, pos.y_m


def fingerprint_match_as_dict(match: FingerprintMatch | None) -> dict | None:
    """JSON-friendly nearest-fingerprint summary for readings / CLI output."""
    if match is None:
        return None
    return {
        "label": match.label,
        "distance_db": match.distance_db,
        "common_aps": match.common_aps,
        "neighbors": list(match.neighbors),
        "blend_weight": match.blend_weight,
        "positioned": match.positioned,
        "position_method": match.position_method,
        "x_m": match.x_m if match.positioned else None,
        "y_m": match.y_m if match.positioned else None,
        "z_m": match.z_m if match.positioned else None,
    }


def fingerprint_ranking_as_dict(
    distance_db: float,
    common_aps: int,
    record: FingerprintRecord,
    *,
    config: LocatorConfig | None = None,
    device_z_m: float | None = None,
    prior_xy: tuple[float, float] | None = None,
) -> dict:
    row: dict = {
        "label": record.label,
        "distance_db": distance_db,
        "common_aps": common_aps,
        "positioned": record.positioned,
    }
    if config is not None:
        effective_z = config.device_z_m if device_z_m is None else device_z_m
        x, y, z, positioned, method = effective_fingerprint_position(
            record,
            config,
            device_z_m=effective_z,
            prior_xy=prior_xy,
        )
        row["positioned"] = positioned
        if method is not None:
            row["position_method"] = method
        if positioned:
            row["x_m"] = x
            row["y_m"] = y
            row["z_m"] = z
    elif record.positioned:
        row["x_m"] = record.x_m
        row["y_m"] = record.y_m
        row["z_m"] = record.z_m
    return row


def build_readings_dict(
    position: PositionReading,
    matched: list[tuple[str, float, float | None]],
    config: LocatorConfig,
    *,
    method: str | None = None,
    fp_match: FingerprintMatch | None = None,
    fingerprint_rankings: list[dict] | None = None,
) -> dict:
    """Full sensor payload for get_readings / local wrapper."""
    payload = position.as_dict()
    payload["access_points"] = access_points_relative_to_position(
        position, matched, config
    )
    if method is not None:
        payload["method"] = method
    fp_dict = fingerprint_match_as_dict(fp_match)
    if fp_dict is not None:
        payload["fingerprint_match"] = fp_dict
        payload["nearest_fingerprint"] = fp_dict["label"]
    if fingerprint_rankings is not None:
        payload["fingerprint_rankings"] = fingerprint_rankings
    return payload


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


def clamp_position_to_floor(
    position: PositionReading,
    config: LocatorConfig,
) -> PositionReading:
    """Clamp to the configured floor extents (reading frame, origin = corner).

    Each axis is clamped to [0, width_m] / [0, height_m] only when that
    dimension is configured; with neither set this is a no-op.
    """
    x, y = position.x_m, position.y_m
    if config.width_m is not None:
        x = min(max(x, 0.0), config.width_m)
    if config.height_m is not None:
        y = min(max(y, 0.0), config.height_m)
    if x == position.x_m and y == position.y_m:
        return position
    return PositionReading(x_m=x, y_m=y, z_m=position.z_m)


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
    dz = current.z_m - previous.z_m
    step_x = dx * alpha
    step_y = dy * alpha
    step_z = dz * alpha

    if max_step_m is not None and max_step_m > 0:
        step = math.sqrt(step_x * step_x + step_y * step_y + step_z * step_z)
        if step > max_step_m:
            scale = max_step_m / step
            step_x *= scale
            step_y *= scale
            step_z *= scale

    return PositionReading(
        x_m=previous.x_m + step_x,
        y_m=previous.y_m + step_y,
        z_m=previous.z_m + step_z,
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
    config: LocatorConfig | None = None,
    device_z_m: float | None = None,
    prior_xy: tuple[float, float] | None = None,
    min_anchors: int = 3,
    max_rssi_delta_db: float | None = 20.0,
    min_rssi_dbm: float = -82.0,
    tx_power_dbm: float = -40.0,
    path_loss_n: float = 2.5,
    weight_temperature: float = 2.0,
) -> FingerprintMatch | None:
    if config is not None and prior_xy is None:
        prior_xy = geometric_centroid_xy(
            config,
            matched,
            device_z_m=device_z_m,
            min_anchors=min_anchors,
            max_rssi_delta_db=max_rssi_delta_db,
            min_rssi_dbm=min_rssi_dbm,
            tx_power_dbm=tx_power_dbm,
            path_loss_n=path_loss_n,
            weight_temperature=weight_temperature,
        )
    rssi_by_ap = matched_to_rssi_dict(matched)
    if not rssi_by_ap:
        return None
    top = store.rank_matches(
        rssi_by_ap,
        k=k,
        min_common_aps=min_common_aps,
        min_common_fraction=min_common_fraction,
        max_rms_db=max_rms_db,
    )
    if not top:
        return None

    best_dist, best_common, best_fp = top[0]
    if device_z_m is not None:
        effective_z = device_z_m
    elif config is not None:
        effective_z = config.device_z_m
    else:
        effective_z = 0.0

    resolved: list[
        tuple[float, int, FingerprintRecord, float, float, float, str | None]
    ] = []
    for dist, common, fp in top:
        if config is not None:
            x, y, z, pos, method = effective_fingerprint_position(
                fp,
                config,
                device_z_m=effective_z,
                prior_xy=prior_xy,
            )
        elif fp.positioned:
            x, y, z, pos, method = fp.x_m, fp.y_m, fp.z_m, True, "stored"
        else:
            continue
        if pos:
            resolved.append((dist, common, fp, x, y, z, method))

    if resolved:
        weights = [1.0 / (d + 0.1) for d, _, _, _, _, _, _ in resolved]
        wsum = sum(weights)
        x = sum(w * px for (_, _, _, px, _, _, _), w in zip(resolved, weights)) / wsum
        y = sum(w * py for (_, _, _, _, py, _, _), w in zip(resolved, weights)) / wsum
        z = sum(w * pz for (_, _, _, _, _, pz, _), w in zip(resolved, weights)) / wsum
        best_method = next(
            (method for _, _, fp, _, _, _, method in resolved if fp.label == best_fp.label),
            resolved[0][6],
        )
        return FingerprintMatch(
            x_m=x,
            y_m=y,
            z_m=z,
            label=best_fp.label,
            distance_db=best_dist,
            common_aps=best_common,
            k=len(top),
            neighbors=tuple(fp.label for _, _, fp in top),
            positioned=True,
            position_method=best_method,
        )

    return FingerprintMatch(
        x_m=0.0,
        y_m=0.0,
        z_m=0.0,
        label=best_fp.label,
        distance_db=best_dist,
        common_aps=best_common,
        k=len(top),
        neighbors=tuple(fp.label for _, _, fp in top),
        positioned=False,
        position_method=None,
    )


def fingerprint_rankings_from_matched(
    store: FingerprintStore,
    matched: list[tuple[str, float, float | None]],
    *,
    k: int = 5,
    min_common_aps: int = 3,
    min_common_fraction: float = 0.5,
    max_rms_db: float | None = None,
    config: LocatorConfig | None = None,
    device_z_m: float | None = None,
    prior_xy: tuple[float, float] | None = None,
    min_anchors: int = 3,
    max_rssi_delta_db: float | None = 20.0,
    min_rssi_dbm: float = -82.0,
    tx_power_dbm: float = -40.0,
    path_loss_n: float = 2.5,
    weight_temperature: float = 2.0,
) -> list[dict]:
    """Top ``k`` stored fingerprints by RSSI similarity to the current scan."""
    if config is not None and prior_xy is None:
        prior_xy = geometric_centroid_xy(
            config,
            matched,
            device_z_m=device_z_m,
            min_anchors=min_anchors,
            max_rssi_delta_db=max_rssi_delta_db,
            min_rssi_dbm=min_rssi_dbm,
            tx_power_dbm=tx_power_dbm,
            path_loss_n=path_loss_n,
            weight_temperature=weight_temperature,
        )
    ranked = store.rank_matches(
        matched_to_rssi_dict(matched),
        k=k,
        min_common_aps=min_common_aps,
        min_common_fraction=min_common_fraction,
        max_rms_db=max_rms_db,
    )
    return [
        fingerprint_ranking_as_dict(
            distance_db,
            common_aps,
            record,
            config=config,
            device_z_m=device_z_m,
            prior_xy=prior_xy,
        )
        for distance_db, common_aps, record in ranked
    ]


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


def range_prior_blend_factor(
    fp_match: FingerprintMatch,
    *,
    max_rms_db: float | None = 10.0,
    min_common_aps: int = 3,
) -> float:
    """
    Scale fingerprint blend for single-range laser constraints.

    Weak RSSI matches keep a low floor (``RANGE_PRIOR_BLEND_FACTOR``); tight
    matches approach full weight so the laser radius can correct geometry range.
    """
    confidence = fingerprint_confidence(
        fp_match,
        max_rms_db=max_rms_db,
        min_common_aps=min_common_aps,
    )
    return RANGE_PRIOR_BLEND_FACTOR + (1.0 - RANGE_PRIOR_BLEND_FACTOR) * confidence


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
    weight = 0.0
    if fp_match.positioned:
        weight = cap * fingerprint_confidence(
            fp_match,
            max_rms_db=max_rms_db,
            min_common_aps=min_common_aps,
        )
        if fp_match.position_method == "range_prior":
            weight *= range_prior_blend_factor(
                fp_match,
                max_rms_db=max_rms_db,
                min_common_aps=min_common_aps,
            )
    blended = PositionReading(
        x_m=centroid.x_m * (1.0 - weight) + fp_match.x_m * weight,
        y_m=centroid.y_m * (1.0 - weight) + fp_match.y_m * weight,
        z_m=centroid.z_m * (1.0 - weight) + fp_match.z_m * weight,
    )
    annotated = FingerprintMatch(
        x_m=fp_match.x_m,
        y_m=fp_match.y_m,
        z_m=fp_match.z_m,
        label=fp_match.label,
        distance_db=fp_match.distance_db,
        common_aps=fp_match.common_aps,
        k=fp_match.k,
        neighbors=fp_match.neighbors,
        blend_weight=weight,
        positioned=fp_match.positioned,
        position_method=fp_match.position_method,
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
    min_anchors: int = 3,
    max_rssi_delta_db: float | None = 20.0,
    min_rssi_dbm: float = -82.0,
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
    fast_scan: bool = True,
    device_z_m: float | None = None,
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
    position, method_used, fp_match = estimate_from_matched(
        config,
        matched,
        min_anchors=min_anchors,
        max_rssi_delta_db=max_rssi_delta_db,
        min_rssi_dbm=min_rssi_dbm,
        tx_power_dbm=tx_power_dbm,
        path_loss_n=path_loss_n,
        weight_temperature=weight_temperature,
        fingerprint_store=fingerprint_store,
        fingerprint_k=fingerprint_k,
        fingerprint_min_common_aps=fingerprint_min_common_aps,
        fingerprint_min_common_fraction=fingerprint_min_common_fraction,
        fingerprint_max_rms_db=fingerprint_max_rms_db,
        fingerprint_max_blend=fingerprint_max_blend,
        device_z_m=device_z_m,
    )
    return position, backend, aggregated, scans_done, method_used, fp_match


def estimate_from_matched(
    config: LocatorConfig,
    matched: list[tuple[str, float, float | None]],
    *,
    min_anchors: int = 3,
    max_rssi_delta_db: float | None = 20.0,
    min_rssi_dbm: float = -82.0,
    tx_power_dbm: float = -40.0,
    path_loss_n: float = 2.5,
    weight_temperature: float = 2.0,
    fingerprint_store: FingerprintStore | None = None,
    fingerprint_k: int = 1,
    fingerprint_min_common_aps: int = 3,
    fingerprint_min_common_fraction: float = 0.5,
    fingerprint_max_rms_db: float | None = 10.0,
    fingerprint_max_blend: float = 0.5,
    device_z_m: float | None = None,
) -> tuple[PositionReading, str, FingerprintMatch | None]:
    """
    Estimate position from already-matched (ap_name, rssi, freq) readings.

    There is one positioning behavior: a weighted centroid (with 3D path-loss
    refinement), blended with a fingerprint match in proportion to its
    confidence whenever the fingerprint store has entries. With an empty (or
    absent) store this is a pure geometric estimate; when geometry fails but a
    fingerprint matches, the fingerprint alone is used.

    Returns (position, method_used, fingerprint_match_or_None) where
    ``method_used`` reports what actually happened: ``weighted_centroid``,
    ``hybrid``, or ``fingerprint``.
    """
    registry = registry_from_config(config)

    effective_device_z = (
        config.device_z_m if device_z_m is None else device_z_m
    )
    estimate = estimate_position(
        registry,
        matched,
        method="weighted_centroid",
        min_anchors=min_anchors,
        device_z_m=effective_device_z,
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
        ).with_z(effective_device_z)

    fp_match: FingerprintMatch | None = None
    if fingerprint_store is not None and fingerprint_store.count() > 0:
        prior_xy = None
        if centroid_position is not None:
            prior_xy = (centroid_position.x_m, centroid_position.y_m)
        # No RMS gate here: confidence weighting in the blend handles poor
        # matches (weight 0 past fingerprint_max_rms_db).
        fp_match = estimate_fingerprint_position(
            fingerprint_store,
            matched,
            k=fingerprint_k,
            min_common_aps=fingerprint_min_common_aps,
            min_common_fraction=fingerprint_min_common_fraction,
            max_rms_db=None,
            config=config,
            device_z_m=effective_device_z,
            prior_xy=prior_xy,
            min_anchors=min_anchors,
            max_rssi_delta_db=max_rssi_delta_db,
            min_rssi_dbm=min_rssi_dbm,
            tx_power_dbm=tx_power_dbm,
            path_loss_n=path_loss_n,
            weight_temperature=weight_temperature,
        )

    # Fingerprints are matched without an RMS gate so the blend can weight
    # them softly — but a fingerprint-only fallback returns the match as-is,
    # so re-apply the gate here to avoid snapping to a garbage match.
    fp_usable_alone = (
        fp_match is not None
        and fp_match.positioned
        and (
            fingerprint_max_rms_db is None
            or fp_match.distance_db <= fingerprint_max_rms_db
        )
    )
    if centroid_position is None and fp_usable_alone:
        assert fp_match is not None
        return (
            clamp_position_to_floor(
                PositionReading(
                    x_m=fp_match.x_m,
                    y_m=fp_match.y_m,
                    z_m=fp_match.z_m,
                ),
                config,
            ),
            "fingerprint",
            fp_match,
        )

    if centroid_position is None:
        raw_anchors = filter_anchors(
            anchors_from_readings(registry, matched),
            min_rssi_dbm=min_rssi_dbm,
        )
        weights = combined_anchor_weights(
            raw_anchors,
            max_delta_db=max_rssi_delta_db,
        )
        usable = [a for a, w in zip(raw_anchors, weights) if w > 0]
        delta_desc = (
            "disabled"
            if max_rssi_delta_db is None
            else f"{max_rssi_delta_db:g} dB below the strongest (soft weighting)"
        )
        fp_note = (
            f" Best fingerprint {fp_match.label!r} was rejected "
            f"(rms {fp_match.distance_db:.1f} dB > "
            f"fingerprint_max_rms_db {fingerprint_max_rms_db:g})."
            if fp_match is not None and fingerprint_max_rms_db is not None
            else ""
        )
        raise RuntimeError(
            f"could not estimate position: matched {len(matched)} AP(s) by BSSID, "
            f"but only {len(usable)} carried enough weight after RSSI filtering "
            f"(need ≥{min_anchors}). Most matched APs were weaker than "
            f"{min_rssi_dbm:g} dBm or more than {delta_desc}. Loosen min_rssi_dbm / "
            f"max_rssi_delta_db, or move closer to more configured APs." + fp_note
        )

    if fp_match is not None:
        blended, fp_match = blend_fingerprint_with_centroid(
            centroid_position,
            fp_match,
            max_blend=fingerprint_max_blend,
            max_rms_db=fingerprint_max_rms_db,
            min_common_aps=fingerprint_min_common_aps,
        )
        return clamp_position_to_floor(blended, config), "hybrid", fp_match

    return clamp_position_to_floor(centroid_position, config), "weighted_centroid", None
