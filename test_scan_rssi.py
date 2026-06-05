#!/usr/bin/env python3
"""
Local test wrapper for WiFi RSSI positioning (same logic as the Viam sensor).

  sudo python3 test_scan_rssi.py
  sudo python3 test_scan_rssi.py --config examples/module_config_viam-5g.json --json
  sudo python3 test_scan_rssi.py --json --debug   # include all BSSIDs heard on SSID

Fingerprint calibration (stand under each AP):

  sudo python3 test_scan_rssi.py --record-fingerprint "Cafe, WoH1"
  sudo python3 test_scan_rssi.py --list-fingerprints
  sudo python3 test_scan_rssi.py --method fingerprint --interval 2
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from time import monotonic
from dataclasses import asdict
from pathlib import Path

from rssi_triangulation.fingerprint import FingerprintStore
from rssi_triangulation.fingerprint_commands import (
    default_fingerprint_db_path,
    execute_fingerprint_command,
)
from rssi_triangulation.fingerprint import FingerprintMatch
from rssi_triangulation.locate import (
    PositionReading,
    build_readings_dict,
    locate_position,
    match_readings_to_aps,
    smooth_position,
)
from rssi_triangulation.module_config import (
    LocatorConfig,
    load_config_file,
    registry_from_config,
)

DEFAULT_CONFIG = Path(__file__).resolve().parent / "examples" / "module_config_viam-5g.json"
_LAST_POSITION: PositionReading | None = None
_FP_STORE: FingerprintStore | None = None
_FP_STORE_PATH: Path | None = None


def effective_fast_scan(args: argparse.Namespace) -> bool:
    return not args.thorough_scan


def effective_scan_delay(args: argparse.Namespace) -> float:
    if args.scan_delay is not None:
        return args.scan_delay
    if effective_fast_scan(args):
        return 0.0
    return 0.15


def effective_min_samples_per_ap(args: argparse.Namespace, scans_done: int) -> int:
    if args.min_samples_per_ap is not None:
        return args.min_samples_per_ap
    return 2 if scans_done >= 3 else 1


def effective_config(args: argparse.Namespace) -> LocatorConfig:
    config = load_config_file(args.config)
    if args.scans is not None:
        return LocatorConfig(
            scan_ssid=config.scan_ssid,
            scan_count=args.scans,
            x_origin_m=config.x_origin_m,
            y_origin_m=config.y_origin_m,
            access_points=config.access_points,
        )
    return config


def fingerprint_db(args: argparse.Namespace) -> FingerprintStore:
    """Reuse one store per DB path (avoids reopening SQLite every --interval tick)."""
    global _FP_STORE, _FP_STORE_PATH
    path = args.fingerprint_db or default_fingerprint_db_path(args.config)
    if _FP_STORE is None or _FP_STORE_PATH != path:
        _FP_STORE = FingerprintStore(path)
        _FP_STORE_PATH = path
    return _FP_STORE


def format_report(
    config: LocatorConfig,
    backend: str,
    matched: list[tuple[str, float, float | None]],
    position: dict[str, float],
    *,
    scans: int,
    scan_delay_s: float,
    heard_count: int,
    method_used: str,
    fp_match: FingerprintMatch | None = None,
    elapsed_s: float | None = None,
) -> str:
    lines = [
        f"SSID: {config.scan_ssid!r}  scans: {scans}",
        f"Floor origin subtracted: x={config.x_origin_m}, y={config.y_origin_m}",
        f"Scan backend: {backend}",
        f"Method: {method_used}",
    ]
    if elapsed_s is not None:
        lines.append(f"Cycle time: {elapsed_s:.2f}s (scans dominate; --interval sleeps after this)")
    if fp_match is not None:
        blend_note = (
            f", blend {fp_match.blend_weight:.0%}"
            if fp_match.blend_weight > 0
            else ""
        )
        lines.append(
            f"Fingerprint: {fp_match.label} (rms {fp_match.distance_db:.1f} dB, "
            f"{fp_match.common_aps} common APs, k={fp_match.k}{blend_note}: "
            f"{', '.join(fp_match.neighbors)})"
        )
    lines.extend(
        [
            f"BSSIDs on network: {heard_count}",
            f"Known APs matched: {len(matched)} / {len(config.access_points)}",
            "",
            "Position:",
            f"  x: {position['location']['x']:.2f} m",
            f"  y: {position['location']['y']:.2f} m",
        ]
    )
    aps = position.get("access_points") or []
    if aps:
        lines.extend(
            [
                "",
                f"{'AP':<16} {'RSSI':>8} {'Δx':>8} {'Δy':>8}",
                "-" * 44,
            ]
        )
        for ap in aps:
            lines.append(
                f"{ap['name']:<16} {ap['rssi']:>8.1f} {ap['x']:>8.2f} {ap['y']:>8.2f}"
            )
    elif matched:
        lines.extend(["", f"{'AP':<16} {'RSSI':>8}", "-" * 28])
        for name, rssi, _ in matched:
            lines.append(f"{name:<16} {rssi:>8.1f}")
    return "\n".join(lines)


def run_fingerprint_cli_action(args: argparse.Namespace) -> int:
    config = effective_config(args)
    db = fingerprint_db(args)

    if args.clear_fingerprints:
        result = execute_fingerprint_command(
            {"command": "clear_fingerprints"},
            config=config,
            db=db,
            interface=args.interface,
            backend=args.backend,
            scan_delay_s=effective_scan_delay(args),
            blocking=args.blocking_scan,
            strict_mac=args.strict_mac,
            min_samples_per_ap=args.min_samples_per_ap,
        )
    elif args.list_fingerprints:
        result = execute_fingerprint_command(
            {"command": "list_fingerprints"},
            config=config,
            db=db,
            interface=args.interface,
            backend=args.backend,
            scan_delay_s=effective_scan_delay(args),
            blocking=args.blocking_scan,
            strict_mac=args.strict_mac,
            min_samples_per_ap=args.min_samples_per_ap,
        )
    elif args.delete_fingerprint:
        result = execute_fingerprint_command(
            {
                "command": "delete_fingerprint",
                "label": args.delete_fingerprint,
            },
            config=config,
            db=db,
            interface=args.interface,
            backend=args.backend,
            scan_delay_s=effective_scan_delay(args),
            blocking=args.blocking_scan,
            strict_mac=args.strict_mac,
            min_samples_per_ap=args.min_samples_per_ap,
        )
    elif args.record_fingerprint:
        result = execute_fingerprint_command(
            {
                "command": "record_fingerprint",
                "ap_name": args.record_fingerprint,
            },
            config=config,
            db=db,
            interface=args.interface,
            backend=args.backend,
            scan_delay_s=effective_scan_delay(args),
            blocking=args.blocking_scan,
            strict_mac=args.strict_mac,
            min_samples_per_ap=args.min_samples_per_ap,
            scan_count_override=args.scans,
        )
    else:
        return 1

    if args.json:
        print(json.dumps(result, indent=2))
    else:
        print(json.dumps(result, indent=2))
    return 0 if result.get("ok") else 1


def run_once(args: argparse.Namespace) -> int:
    global _LAST_POSITION
    t0 = monotonic()
    config = effective_config(args)
    fp_store = (
        fingerprint_db(args) if args.method in ("fingerprint", "hybrid") else None
    )

    raw_position, backend, readings, scans_done, method_used, fp_match = locate_position(
        config,
        interface=args.interface,
        backend=args.backend,
        scan_delay_s=effective_scan_delay(args),
        blocking=args.blocking_scan,
        strict_mac=args.strict_mac,
        method=args.method,
        min_anchors=args.min_aps,
        max_rssi_delta_db=None if args.no_rssi_filter else args.max_rssi_delta,
        min_rssi_dbm=args.min_rssi,
        min_samples_per_ap=args.min_samples_per_ap,
        tx_power_dbm=args.tx_power,
        path_loss_n=args.path_loss_n,
        weight_temperature=args.weight_temperature,
        fingerprint_store=fp_store,
        fingerprint_k=args.fingerprint_k,
        fingerprint_min_common_aps=args.fingerprint_min_common_aps,
        fingerprint_min_common_fraction=args.fingerprint_min_common_fraction,
        fingerprint_max_rms_db=(
            None if args.no_fingerprint_max_rms else args.fingerprint_max_rms
        ),
        fingerprint_max_blend=args.fingerprint_max_blend,
        fingerprint_fallback=not args.no_fingerprint_fallback,
        fast_scan=effective_fast_scan(args),
    )
    elapsed_s = monotonic() - t0
    position = smooth_position(
        _LAST_POSITION,
        raw_position,
        alpha=args.smoothing_alpha,
        max_step_m=None if args.max_position_step_m <= 0 else args.max_position_step_m,
    )
    _LAST_POSITION = position
    registry = registry_from_config(config)
    matched = match_readings_to_aps(
        readings,
        registry,
        strict_mac=args.strict_mac,
        min_sample_count=effective_min_samples_per_ap(args, scans_done),
    )
    payload = build_readings_dict(position, matched, config)
    raw_payload = build_readings_dict(raw_position, matched, config)

    if args.json:
        out: dict = {
            "config": str(args.config),
            "backend": backend,
            "method": method_used,
            "fingerprint_match": (
                {
                    "label": fp_match.label,
                    "distance_db": fp_match.distance_db,
                    "common_aps": fp_match.common_aps,
                    "neighbors": list(fp_match.neighbors),
                    "blend_weight": fp_match.blend_weight,
                }
                if fp_match is not None
                else None
            ),
            "scans": scans_done,
            "fingerprint_db": str(fp_store.path) if fp_store else None,
            "fingerprint_count": fp_store.count() if fp_store else None,
            "min_samples_per_ap": effective_min_samples_per_ap(args, scans_done),
            "matched_aps": len(matched),
            "readings": payload,
            "elapsed_s": round(elapsed_s, 3),
        }
        if args.debug:
            out["raw_readings"] = raw_payload
            out["heard"] = [asdict(r) for r in readings]
            out["matched"] = [
                {"name": name, "rssi_dbm": rssi, "frequency_mhz": freq}
                for name, rssi, freq in matched
            ]
        print(json.dumps(out, indent=2))
    else:
        print(
            format_report(
                config,
                backend,
                matched,
                payload,
                scans=scans_done,
                scan_delay_s=effective_scan_delay(args),
                heard_count=len(readings),
                method_used=method_used,
                fp_match=fp_match,
                elapsed_s=elapsed_s,
            )
        )
        if method_used != args.method and args.method in ("fingerprint", "hybrid"):
            print(
                f"  (requested {args.method}; used {method_used})",
                file=sys.stderr,
            )
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Scan WiFi RSSI and estimate floor-plan position (local wrapper)."
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG,
        help=f"Module config JSON (default: {DEFAULT_CONFIG.name})",
    )
    parser.add_argument("--interface", default=None)
    parser.add_argument(
        "--backend",
        choices=["iw", "wpa_cli", "nmcli"],
        default=None,
    )
    parser.add_argument(
        "--scans",
        type=int,
        default=None,
        metavar="N",
        help="Override scan_count from config",
    )
    parser.add_argument("--scan-delay", type=float, default=None, metavar="SEC")
    parser.add_argument(
        "--thorough-scan",
        action="store_true",
        help="Slower scans: iw long poll, nmcli fallback, delay between passes (max RSSI stability)",
    )
    parser.add_argument("--blocking-scan", action="store_true")
    parser.add_argument("--interval", type=float, default=0.0, metavar="SEC")
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--json", action="store_true")
    parser.add_argument(
        "--debug",
        action="store_true",
        help="With --json: include heard (all BSSIDs on SSID) and matched AP RSSI details",
    )
    parser.add_argument("--strict-mac", action="store_true", default=True)
    parser.add_argument("--no-strict-mac", action="store_false", dest="strict_mac")
    parser.add_argument("--max-rssi-delta", type=float, default=35.0, metavar="DB")
    parser.add_argument("--min-rssi", type=float, default=-90.0, metavar="DBM")
    parser.add_argument("--no-rssi-filter", action="store_true")
    parser.add_argument(
        "--method",
        choices=["weighted_centroid", "path_loss", "fingerprint", "hybrid"],
        default="hybrid",
        help=(
            "hybrid (default) blends weighted centroid with fingerprints; "
            "weighted_centroid = geometry only"
        ),
    )
    parser.add_argument("--min-aps", type=int, default=3, metavar="N")
    parser.add_argument(
        "--min-samples-per-ap",
        type=int,
        default=None,
        metavar="N",
        help="Require AP to appear in at least N scan passes (default: 2 when scans >= 3, else 1)",
    )
    parser.add_argument("--tx-power", type=float, default=-40.0)
    parser.add_argument("--path-loss-n", type=float, default=2.5, dest="path_loss_n")
    parser.add_argument(
        "--weight-temperature",
        type=float,
        default=2.0,
        metavar="T",
        help=(
            "Softens RSSI weighting for the centroid: 1=winner-take-all (jumpy), "
            "higher=flatter/steadier (default: 2)"
        ),
    )
    parser.add_argument(
        "--smoothing-alpha",
        type=float,
        default=1.0,
        metavar="A",
        help="Temporal smoothing alpha for repeated readings (0=freeze, 1=no smoothing; default: 1)",
    )
    parser.add_argument(
        "--max-position-step-m",
        type=float,
        default=0.0,
        metavar="M",
        help="Limit smoothed movement per reading; <=0 disables (default: disabled)",
    )
    fp = parser.add_argument_group("fingerprint calibration")
    fp.add_argument(
        "--fingerprint-db",
        type=Path,
        default=None,
        help="SQLite DB path (default: <config-dir>/fingerprints.sqlite)",
    )
    fp.add_argument(
        "--record-fingerprint",
        metavar="AP_NAME",
        help="Stand under this AP and record its RSSI fingerprint, then exit",
    )
    fp.add_argument(
        "--list-fingerprints",
        action="store_true",
        help="List stored fingerprints and exit",
    )
    fp.add_argument(
        "--delete-fingerprint",
        metavar="LABEL",
        help="Delete one fingerprint by label and exit",
    )
    fp.add_argument(
        "--clear-fingerprints",
        action="store_true",
        help="Delete all fingerprints and exit",
    )
    fp.add_argument("--fingerprint-k", type=int, default=1, metavar="K")
    fp.add_argument(
        "--fingerprint-min-common-aps",
        type=int,
        default=3,
        metavar="N",
        help="Minimum overlapping APs for a fingerprint match (default: 3)",
    )
    fp.add_argument(
        "--fingerprint-min-common-fraction",
        type=float,
        default=0.5,
        metavar="F",
        help="Also require this fraction of the smaller AP set to overlap (default: 0.5)",
    )
    fp.add_argument(
        "--fingerprint-max-rms",
        type=float,
        default=10.0,
        metavar="DB",
        help="Max normalized RMS RSSI error to accept a match (default: 10)",
    )
    fp.add_argument(
        "--no-fingerprint-max-rms",
        action="store_true",
        help="Disable max RMS gate on fingerprint matching",
    )
    fp.add_argument(
        "--fingerprint-max-blend",
        type=float,
        default=0.5,
        metavar="W",
        help="For --method hybrid: max fingerprint weight 0–1 (default: 0.5)",
    )
    fp.add_argument(
        "--no-fingerprint-fallback",
        action="store_true",
        help="With --method fingerprint, do not fall back to weighted centroid",
    )
    args = parser.parse_args()

    if args.scans is not None and args.scans < 1:
        parser.error("--scans must be >= 1")
    if args.min_samples_per_ap is not None and args.min_samples_per_ap < 1:
        parser.error("--min-samples-per-ap must be >= 1")
    if not 0 <= args.smoothing_alpha <= 1:
        parser.error("--smoothing-alpha must be between 0 and 1")

    fp_actions = (
        args.record_fingerprint
        or args.list_fingerprints
        or args.delete_fingerprint
        or args.clear_fingerprints
    )
    if fp_actions:
        try:
            return run_fingerprint_cli_action(args)
        except (RuntimeError, ValueError) as exc:
            print(f"error: {exc}", file=sys.stderr)
            return 1

    continuous = args.interval > 0 and not args.once
    if not continuous:
        try:
            return run_once(args)
        except (RuntimeError, ValueError) as exc:
            print(f"error: {exc}", file=sys.stderr)
            return 1

    db_path = args.fingerprint_db or default_fingerprint_db_path(args.config)
    print(
        f"Scanning every {args.interval}s after each cycle completes ({args.config})\n"
        f"  method={args.method} scans={args.scans or 'from config'} "
        f"fast_scan={effective_fast_scan(args)} fingerprints={db_path}\n"
        f"  (--interval is not the scan period; one cycle is ~3× WiFi scan time)\n"
        f"  Ctrl+C to stop\n"
    )
    try:
        while True:
            try:
                run_once(args)
            except (RuntimeError, ValueError) as exc:
                print(f"error: {exc}", file=sys.stderr)
            print()
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\nStopped.")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
