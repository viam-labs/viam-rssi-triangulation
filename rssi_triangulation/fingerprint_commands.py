"""Shared do_command / CLI handlers for fingerprint calibration."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

from .calibrate import calibrate_from_fingerprints
from .fingerprint import FingerprintStore, matched_to_rssi_dict
from .locate import collect_matched_scan
from .module_config import LocatorConfig


def default_fingerprint_db_path(config_path: Path | None = None) -> Path:
    if config_path is not None:
        return config_path.parent / "fingerprints.sqlite"
    return Path("fingerprints.sqlite")


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
      list_fingerprints — { "command": "list_fingerprints" }
      delete_fingerprint — { "command": "delete_fingerprint", "label": "..." }
      clear_fingerprints — { "command": "clear_fingerprints" }
      calibrate_path_loss — { "command": "calibrate_path_loss" } fits
          tx_power_dbm / path_loss_n from stored fingerprints
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
                    "x_m": r.x_m,
                    "y_m": r.y_m,
                    "z_m": r.z_m,
                    "ap_count": len(r.rssi_by_ap),
                    "recorded_at": r.recorded_at,
                    "scan_count": r.scan_count,
                }
                for r in records
            ],
        }

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

    raise ValueError(
        f"unknown fingerprint command {name!r}; supported: record_fingerprint, "
        "record_fingerprint_here, list_fingerprints, delete_fingerprint, "
        "clear_fingerprints, calibrate_path_loss"
    )


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
        x_m, y_m, z_m = ap_position_in_reading_frame(config, ap_name)
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
