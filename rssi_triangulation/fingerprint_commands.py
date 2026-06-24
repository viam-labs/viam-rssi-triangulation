"""Shared do_command / CLI handlers for fingerprint calibration."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

from .calibrate import MIN_SAMPLE_DISTANCE_M, calibrate_from_fingerprints
from .geo import infer_xy_from_slant_distances
from .fingerprint import FingerprintStore, matched_to_rssi_dict
from .fingerprint_session import get_fingerprint_session_manager
from .locate import (
    ap_position_in_reading_frame,
    collect_matched_scan,
    estimate_fingerprint_position,
    fingerprint_match_as_dict,
    fingerprint_rankings_from_matched,
    geometric_centroid_xy,
    infer_fingerprint_xy_at_record,
)
from .module_config import LocatorConfig


def default_fingerprint_db_path(config_path: Path | None = None) -> Path:
    if config_path is not None:
        return config_path.parent / "fingerprints.sqlite"
    return Path("fingerprints.sqlite")


def parse_distances_m(
    command: Mapping[str, Any],
    config: LocatorConfig,
) -> dict[str, float]:
    """Parse optional laser/measured distances keyed by configured AP name."""
    distances: dict[str, float] = {}
    raw_map = command.get("distances_m")
    if isinstance(raw_map, Mapping):
        for ap_name, value in raw_map.items():
            if not isinstance(ap_name, str) or not ap_name:
                raise ValueError("distances_m keys must be AP name strings")
            distances[ap_name] = float(value)

    raw_list = command.get("distance_to_ap")
    if raw_list is not None:
        if not isinstance(raw_list, list):
            raise ValueError('distance_to_ap must be a list of {"ap_name", "distance_m"} objects')
        for item in raw_list:
            if not isinstance(item, Mapping):
                raise ValueError('distance_to_ap entries must be objects with ap_name and distance_m')
            ap_name = item.get("ap_name")
            if not isinstance(ap_name, str) or not ap_name:
                raise ValueError('distance_to_ap entries require "ap_name"')
            if "distance_m" not in item:
                raise ValueError('distance_to_ap entries require "distance_m"')
            distances[ap_name] = float(item["distance_m"])

    for ap_name, distance_m in distances.items():
        ap_position_in_reading_frame(config, ap_name)
        if distance_m < MIN_SAMPLE_DISTANCE_M:
            raise ValueError(
                f"distance to {ap_name!r} ({distance_m:.2f} m) is below the "
                f"minimum {MIN_SAMPLE_DISTANCE_M} m"
            )
    return distances


def execute_fingerprint_command(
    command: Mapping[str, Any],
    *,
    config: LocatorConfig,
    db: FingerprintStore,
    interface: str | None = None,
    backend: str | None = None,
    scan_delay_s: float = 0.15,
    blocking: bool = False,
    strict_mac: bool = True,
    min_samples_per_ap: int | None = None,
    scan_count_override: int | None = None,
    device_z_m: float | None = None,
    fast_scan: bool = True,
    current_tx_power_dbm: float | None = None,
    current_path_loss_n: float | None = None,
) -> dict[str, Any]:
    """
    Run a fingerprint calibration command.

    Commands:
      record_fingerprint — { "command": "record_fingerprint", "ap_name": "..." }
      record_fingerprint_here — { "command": "record_fingerprint_here", "label": "...",
          "x_m": float, "y_m": float, optional "z_m": float }
      record_fingerprint_rssi — { "command": "record_fingerprint_rssi", "label": "...",
          optional "distances_m": { "SoA1": 8.3 } or "distance_to_ap": [
              { "ap_name": "SoA1", "distance_m": 8.3 } ] }
      list_fingerprints — { "command": "list_fingerprints" }
      delete_fingerprint — { "command": "delete_fingerprint", "label": "..." }
      clear_fingerprints — { "command": "clear_fingerprints" }
      calibrate_path_loss — { "command": "calibrate_path_loss" } fits
          tx_power_dbm / path_loss_n from stored fingerprints
      start_fingerprint_recording — begin accumulating scans (see stop_*)
      sample_fingerprint_recording — add one scan to the active session
      fingerprint_recording_status — sample count and session metadata
      stop_fingerprint_recording — aggregate samples (mean/std per AP) and store
      cancel_fingerprint_recording — discard the active session
    """
    name = command.get("command")
    if not isinstance(name, str) or not name:
        raise ValueError('command must include a non-empty "command" field')

    if name == "list_fingerprints":
        records = db.list_all()
        return {
            "ok": True,
            "command": name,
            "db_path": str(db.path),
            "count": len(records),
            "fingerprints": [
                {
                    "label": r.label,
                    "positioned": r.positioned,
                    "x_m": r.x_m if r.positioned else None,
                    "y_m": r.y_m if r.positioned else None,
                    "z_m": r.z_m if r.positioned else None,
                    "distances_m": r.distances_by_ap or {},
                    "ap_count": len(r.rssi_by_ap),
                    "recorded_at": r.recorded_at,
                    "scan_count": r.scan_count,
                    "rssi_stats_by_ap": r.rssi_stats_by_ap or {},
                }
                for r in records
            ],
        }

    if name == "fingerprint_recording_status":
        status = get_fingerprint_session_manager().status()
        return {"ok": True, "command": name, **status}

    if name == "cancel_fingerprint_recording":
        result = get_fingerprint_session_manager().cancel()
        result["command"] = name
        return result

    if name == "start_fingerprint_recording":
        return _start_fingerprint_recording(command, config=config)

    if name == "sample_fingerprint_recording":
        return _sample_fingerprint_recording(
            command,
            config=config,
            interface=interface,
            backend=backend,
            scan_delay_s=scan_delay_s,
            blocking=blocking,
            strict_mac=strict_mac,
            min_samples_per_ap=min_samples_per_ap,
            scan_count_override=scan_count_override,
            fast_scan=fast_scan,
            device_z_m=device_z_m,
        )

    if name == "stop_fingerprint_recording":
        return _stop_fingerprint_recording(
            command,
            config=config,
            db=db,
            interface=interface,
            backend=backend,
            scan_delay_s=scan_delay_s,
            blocking=blocking,
            strict_mac=strict_mac,
            min_samples_per_ap=min_samples_per_ap,
            scan_count_override=scan_count_override,
            device_z_m=device_z_m,
            fast_scan=fast_scan,
        )

    if name == "delete_fingerprint":
        label = command.get("label") or command.get("ap_name")
        if not isinstance(label, str) or not label:
            raise ValueError('delete_fingerprint requires "label" or "ap_name"')
        deleted = db.delete(label)
        return {"ok": deleted, "command": name, "label": label, "deleted": deleted}

    if name == "clear_fingerprints":
        removed = db.clear()
        return {"ok": True, "command": name, "removed": removed, "db_path": str(db.path)}

    if name == "calibrate_path_loss":
        result = calibrate_from_fingerprints(
            config,
            db,
            current_tx_power_dbm=current_tx_power_dbm,
            current_path_loss_n=current_path_loss_n,
        )
        result["command"] = name
        return result

    if name == "match_fingerprints":
        return _match_fingerprints(
            command,
            config=config,
            db=db,
            interface=interface,
            backend=backend,
            scan_delay_s=scan_delay_s,
            blocking=blocking,
            strict_mac=strict_mac,
            min_samples_per_ap=min_samples_per_ap,
            scan_count_override=scan_count_override,
            fast_scan=fast_scan,
        )

    if name in ("record_fingerprint", "record_fingerprint_here"):
        return _record_fingerprint(
            name,
            command,
            config=config,
            db=db,
            interface=interface,
            backend=backend,
            scan_delay_s=scan_delay_s,
            blocking=blocking,
            strict_mac=strict_mac,
            min_samples_per_ap=min_samples_per_ap,
            scan_count_override=scan_count_override,
            device_z_m=device_z_m,
            fast_scan=fast_scan,
        )

    if name == "record_fingerprint_rssi":
        return _record_fingerprint_rssi(
            command,
            config=config,
            db=db,
            interface=interface,
            backend=backend,
            scan_delay_s=scan_delay_s,
            blocking=blocking,
            strict_mac=strict_mac,
            min_samples_per_ap=min_samples_per_ap,
            scan_count_override=scan_count_override,
            fast_scan=fast_scan,
        )

    raise ValueError(
        f"unknown fingerprint command {name!r}; supported: record_fingerprint, "
        "record_fingerprint_here, record_fingerprint_rssi, start_fingerprint_recording, "
        "sample_fingerprint_recording, fingerprint_recording_status, "
        "stop_fingerprint_recording, cancel_fingerprint_recording, match_fingerprints, "
        "list_fingerprints, delete_fingerprint, clear_fingerprints, calibrate_path_loss"
    )


def _start_fingerprint_recording(
    command: Mapping[str, Any],
    *,
    config: LocatorConfig,
) -> dict[str, Any]:
    label = command.get("label")
    if label is not None and (not isinstance(label, str) or not label):
        raise ValueError('"label" must be a non-empty string when provided')
    distances_by_ap = {}
    if command.get("distances_m") or command.get("distance_to_ap"):
        distances_by_ap = parse_distances_m(command, config)
    min_samples = int(command.get("min_samples", 5))
    auto_sample = command.get("auto_sample", True)
    if not isinstance(auto_sample, bool):
        auto_sample = bool(auto_sample)
    positioned = bool(command.get("positioned", False))
    x_m = y_m = z_m = None
    if "x_m" in command and "y_m" in command:
        x_m = float(command["x_m"])
        y_m = float(command["y_m"])
        positioned = True
        if "z_m" in command:
            z_m = float(command["z_m"])
    status = get_fingerprint_session_manager().start(
        label=label,
        x_m=x_m,
        y_m=y_m,
        z_m=z_m,
        positioned=positioned,
        distances_by_ap=distances_by_ap or None,
        min_samples=min_samples,
        auto_sample=auto_sample,
    )
    return {
        "ok": True,
        "command": "start_fingerprint_recording",
        **status,
        "hint": (
            "While recording: keep get_readings() or test_scan_rssi.py running "
            "(auto_sample=true), or call sample_fingerprint_recording periodically. "
            "Then stop_fingerprint_recording."
        ),
    }


def _sample_fingerprint_recording(
    command: Mapping[str, Any],
    *,
    config: LocatorConfig,
    interface: str | None,
    backend: str | None,
    scan_delay_s: float,
    blocking: bool,
    strict_mac: bool,
    min_samples_per_ap: int | None,
    scan_count_override: int | None,
    fast_scan: bool,
    device_z_m: float | None = None,
) -> dict[str, Any]:
    manager = get_fingerprint_session_manager()
    if not manager.active():
        raise RuntimeError("no active fingerprint recording session")
    effective_scans = scan_count_override if scan_count_override is not None else config.scan_count
    matched, backend_name, _aggregated, scans_done = collect_matched_scan(
        config,
        interface=interface,
        backend=backend,
        scan_delay_s=scan_delay_s,
        blocking=blocking,
        strict_mac=strict_mac,
        min_samples_per_ap=min_samples_per_ap,
        scan_count_override=effective_scans,
        fast_scan=fast_scan,
    )
    prior_xy = geometric_centroid_xy(
        config,
        matched,
        device_z_m=device_z_m,
        min_anchors=2,
    )
    count = manager.add_sample(
        matched_to_rssi_dict(matched),
        prior_xy=prior_xy,
    )
    return {
        "ok": True,
        "command": "sample_fingerprint_recording",
        "sample_count": count,
        "bssids_heard": len(matched),
        "backend": backend_name,
        "scans": scans_done,
        **manager.status(),
    }


def _stop_fingerprint_recording(
    command: Mapping[str, Any],
    *,
    config: LocatorConfig,
    db: FingerprintStore,
    interface: str | None,
    backend: str | None,
    scan_delay_s: float,
    blocking: bool,
    strict_mac: bool,
    min_samples_per_ap: int | None,
    scan_count_override: int | None,
    device_z_m: float | None,
    fast_scan: bool,
) -> dict[str, Any]:
    manager = get_fingerprint_session_manager()
    if not manager.active():
        raise RuntimeError("no active fingerprint recording session")

    extra_scans = command.get("scan_count")
    if extra_scans is not None:
        matched, _backend_name, _aggregated, _scans_done = collect_matched_scan(
            config,
            interface=interface,
            backend=backend,
            scan_delay_s=scan_delay_s,
            blocking=blocking,
            strict_mac=strict_mac,
            min_samples_per_ap=min_samples_per_ap,
            scan_count_override=int(extra_scans),
            fast_scan=fast_scan,
        )
        manager.add_sample(matched_to_rssi_dict(matched))

    distances_override = None
    if command.get("distances_m") or command.get("distance_to_ap"):
        distances_override = parse_distances_m(command, config)

    label = command.get("label")
    if label is not None and (not isinstance(label, str) or not label):
        raise ValueError('"label" must be a non-empty string when provided')

    positioned_arg = None
    if "positioned" in command:
        positioned_arg = bool(command["positioned"])
    x_m = float(command["x_m"]) if "x_m" in command else None
    y_m = float(command["y_m"]) if "y_m" in command else None
    z_m = float(command["z_m"]) if "z_m" in command else None
    if x_m is not None and y_m is not None:
        positioned_arg = True

    session, means, stats = manager.stop(
        label=label,
        x_m=x_m,
        y_m=y_m,
        z_m=z_m,
        positioned=positioned_arg,
        distances_by_ap=distances_override,
    )

    effective_z = config.device_z_m if device_z_m is None else device_z_m
    distances = dict(session.distances_by_ap)
    missing_rssi = sorted(set(distances) - set(means))
    if missing_rssi:
        raise ValueError(
            "distance_to_ap includes APs not heard during recording: "
            + ", ".join(missing_rssi)
        )

    positioned = session.positioned
    x_out = session.x_m or 0.0
    y_out = session.y_m or 0.0
    z_out = session.z_m if session.z_m is not None else effective_z
    if x_m is not None and y_m is not None:
        x_out, y_out = x_m, y_m
        positioned = True
    elif not positioned and distances:
        frozen = infer_fingerprint_xy_at_record(
            config,
            distances_by_ap=distances,
            prior_xy_samples=session.prior_xy_samples,
            device_z_m=effective_z,
        )
        if frozen is not None:
            x_out, y_out = frozen
            positioned = True

    record = db.record(
        session.label or "",
        x_m=x_out,
        y_m=y_out,
        z_m=z_out,
        rssi_by_ap=means,
        scan_count=len(session.samples),
        positioned=positioned,
        distances_by_ap=distances or None,
        rssi_stats_by_ap=stats,
    )
    return {
        "ok": True,
        "command": "stop_fingerprint_recording",
        "label": record.label,
        "positioned": positioned,
        "x_m": record.x_m if positioned else None,
        "y_m": record.y_m if positioned else None,
        "z_m": record.z_m if positioned else None,
        "distances_m": record.distances_by_ap or {},
        "ap_rssi": record.rssi_by_ap,
        "rssi_stats_by_ap": record.rssi_stats_by_ap or {},
        "sample_count": len(session.samples),
        "prior_xy_samples": len(session.prior_xy_samples),
        "position_frozen_at_record": positioned and x_m is None and y_m is None,
        "db_path": str(db.path),
        "recorded_at": record.recorded_at,
    }


def _match_fingerprints(
    command: Mapping[str, Any],
    *,
    config: LocatorConfig,
    db: FingerprintStore,
    interface: str | None,
    backend: str | None,
    scan_delay_s: float,
    blocking: bool,
    strict_mac: bool,
    min_samples_per_ap: int | None,
    scan_count_override: int | None,
    fast_scan: bool,
) -> dict[str, Any]:
    rank_k = int(command.get("k", 5))
    if rank_k < 1:
        raise ValueError("k must be >= 1")
    min_common_aps = int(command.get("min_common_aps", 3))
    min_common_fraction = float(command.get("min_common_fraction", 0.5))
    max_rms_db = command.get("max_rms_db")
    max_rms = float(max_rms_db) if max_rms_db is not None else None

    scan_count = scan_count_override
    if scan_count is None and "scan_count" in command:
        scan_count = int(command["scan_count"])
    if scan_count is not None and scan_count < 1:
        raise ValueError("scan_count must be >= 1")

    matched, backend_name, _aggregated, scans_done = collect_matched_scan(
        config,
        interface=interface,
        backend=backend,
        scan_delay_s=scan_delay_s,
        blocking=blocking,
        strict_mac=strict_mac,
        min_samples_per_ap=min_samples_per_ap,
        scan_count_override=scan_count,
        fast_scan=fast_scan,
    )
    fp_match = estimate_fingerprint_position(
        db,
        matched,
        k=1,
        min_common_aps=min_common_aps,
        min_common_fraction=min_common_fraction,
        max_rms_db=max_rms,
        config=config,
        device_z_m=device_z_m,
    )
    rankings = fingerprint_rankings_from_matched(
        db,
        matched,
        k=rank_k,
        min_common_aps=min_common_aps,
        min_common_fraction=min_common_fraction,
        max_rms_db=max_rms,
        config=config,
        device_z_m=device_z_m,
    )
    return {
        "ok": True,
        "command": "match_fingerprints",
        "nearest_fingerprint": fp_match.label if fp_match is not None else None,
        "fingerprint_match": fingerprint_match_as_dict(fp_match),
        "fingerprint_rankings": rankings,
        "bssids_heard": len(matched),
        "backend": backend_name,
        "scans": scans_done,
        "db_path": str(db.path),
    }


def _record_fingerprint(
    cmd: str,
    command: Mapping[str, Any],
    *,
    config: LocatorConfig,
    db: FingerprintStore,
    interface: str | None,
    backend: str | None,
    scan_delay_s: float,
    blocking: bool,
    strict_mac: bool,
    min_samples_per_ap: int | None,
    scan_count_override: int | None,
    device_z_m: float | None,
    fast_scan: bool,
) -> dict[str, Any]:
    effective_device_z = (
        config.device_z_m if device_z_m is None else device_z_m
    )
    if cmd == "record_fingerprint":
        ap_name = command.get("ap_name") or command.get("label")
        if not isinstance(ap_name, str) or not ap_name:
            raise ValueError('record_fingerprint requires "ap_name"')
        x_m, y_m, _ = ap_position_in_reading_frame(config, ap_name)
        z_m = effective_device_z
        label = ap_name
    else:
        label = command.get("label")
        if not isinstance(label, str) or not label:
            raise ValueError('record_fingerprint_here requires "label"')
        if "x_m" not in command or "y_m" not in command:
            raise ValueError('record_fingerprint_here requires "x_m" and "y_m"')
        x_m = float(command["x_m"])
        y_m = float(command["y_m"])
        z_m = (
            float(command["z_m"])
            if "z_m" in command
            else effective_device_z
        )

    scan_count = scan_count_override
    if scan_count is None and "scan_count" in command:
        scan_count = int(command["scan_count"])
    if scan_count is not None and scan_count < 1:
        raise ValueError("scan_count must be >= 1")

    matched, backend_name, _aggregated, scans_done = collect_matched_scan(
        config,
        interface=interface,
        backend=backend,
        scan_delay_s=scan_delay_s,
        blocking=blocking,
        strict_mac=strict_mac,
        min_samples_per_ap=min_samples_per_ap,
        scan_count_override=scan_count,
        fast_scan=fast_scan,
    )
    rssi_by_ap = matched_to_rssi_dict(matched)
    record = db.record(
        label,
        x_m=x_m,
        y_m=y_m,
        z_m=z_m,
        rssi_by_ap=rssi_by_ap,
        scan_count=scans_done,
    )
    return {
        "ok": True,
        "command": cmd,
        "label": record.label,
        "x_m": record.x_m,
        "y_m": record.y_m,
        "z_m": record.z_m,
        "bssids_heard": len(matched),
        "ap_rssi": record.rssi_by_ap,
        "backend": backend_name,
        "scans": scans_done,
        "db_path": str(db.path),
        "recorded_at": record.recorded_at,
    }


def _record_fingerprint_rssi(
    command: Mapping[str, Any],
    *,
    config: LocatorConfig,
    db: FingerprintStore,
    interface: str | None,
    backend: str | None,
    scan_delay_s: float,
    blocking: bool,
    strict_mac: bool,
    min_samples_per_ap: int | None,
    scan_count_override: int | None,
    fast_scan: bool,
) -> dict[str, Any]:
    label = command.get("label")
    if not isinstance(label, str) or not label:
        raise ValueError('record_fingerprint_rssi requires "label"')

    distances_by_ap = parse_distances_m(command, config)

    scan_count = scan_count_override
    if scan_count is None and "scan_count" in command:
        scan_count = int(command["scan_count"])
    if scan_count is not None and scan_count < 1:
        raise ValueError("scan_count must be >= 1")

    matched, backend_name, _aggregated, scans_done = collect_matched_scan(
        config,
        interface=interface,
        backend=backend,
        scan_delay_s=scan_delay_s,
        blocking=blocking,
        strict_mac=strict_mac,
        min_samples_per_ap=min_samples_per_ap,
        scan_count_override=scan_count,
        fast_scan=fast_scan,
    )
    rssi_by_ap = matched_to_rssi_dict(matched)
    missing_rssi = sorted(set(distances_by_ap) - set(rssi_by_ap))
    if missing_rssi:
        raise ValueError(
            "distance_to_ap includes APs not heard in this scan: "
            + ", ".join(missing_rssi)
        )

    device_z_m = config.device_z_m
    positioned = False
    x_m = y_m = 0.0
    inferred_position = False
    if len(distances_by_ap) >= 2:
        ap_ranges = {
            ap_name: (*ap_position_in_reading_frame(config, ap_name), slant)
            for ap_name, slant in distances_by_ap.items()
        }
        xy = infer_xy_from_slant_distances(
            ap_ranges,
            device_z_m=device_z_m,
            config=config,
            min_horizontal_m=MIN_SAMPLE_DISTANCE_M,
        )
        if xy is not None:
            x_m, y_m = xy
            positioned = True
            inferred_position = True

    record = db.record(
        label,
        x_m=x_m,
        y_m=y_m,
        z_m=device_z_m,
        rssi_by_ap=rssi_by_ap,
        scan_count=scans_done,
        positioned=positioned,
        distances_by_ap=distances_by_ap or None,
    )
    return {
        "ok": True,
        "command": "record_fingerprint_rssi",
        "label": record.label,
        "positioned": positioned,
        "inferred_position": inferred_position,
        "x_m": record.x_m if positioned else None,
        "y_m": record.y_m if positioned else None,
        "z_m": record.z_m if positioned else None,
        "distances_m": record.distances_by_ap or {},
        "bssids_heard": len(matched),
        "ap_rssi": record.rssi_by_ap,
        "backend": backend_name,
        "scans": scans_done,
        "db_path": str(db.path),
        "recorded_at": record.recorded_at,
    }
