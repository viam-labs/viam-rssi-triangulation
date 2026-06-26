"""Parse Viam module / local JSON config for the WiFi position sensor."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

from .aps import normalize_mac
from .registry import AccessPoint, ApRegistry


@dataclass(frozen=True)
class ConfiguredAccessPoint:
    name: str
    x_m: float
    y_m: float
    z_m: float
    bssid: str


@dataclass(frozen=True)
class BleBeacon:
    """A BLE beacon at a fixed floor-plan position used for ranging.

    ``mac_address`` should be a lowercase colon-separated MAC.
    ``tx_power_dbm`` is the transmit power at 1 m (calibrate per device;
    iBeacon standard is typically −59 dBm).  ``path_loss_n`` is the
    environment-specific path-loss exponent (2–4; same model as WiFi).
    """

    name: str
    x_m: float
    y_m: float
    z_m: float
    mac_address: str
    tx_power_dbm: float = -59.0
    path_loss_n: float = 2.5


@dataclass(frozen=True)
class LocatorConfig:
    # Floor plan geometry (shared by all signal sources)
    x_origin_m: float = 0.0
    y_origin_m: float = 0.0
    device_z_m: float = 0.0
    access_point_z_m: float = 0.0
    # WiFi scanning — required only when WiFi is enabled
    scan_ssid: str = ""
    scan_count: int = 1
    access_points: tuple[ConfiguredAccessPoint, ...] = ()
    # Optional floor extents in the reading frame (origin = corner of the
    # floor). When set, positions are clamped to [0, width] / [0, height].
    width_m: float | None = None
    height_m: float | None = None
    # BLE beacons for ranging / sensor fusion
    ble_beacons: tuple[BleBeacon, ...] = ()
    ble_min_rssi_dbm: float = -90.0
    # Explicit signal-source enable flags.  When True (the default) each source
    # is active if it has configured items (access_points / ble_beacons).
    # Set to False to disable a source even when items are present — useful for
    # temporarily testing one source while keeping the other's config intact.
    wifi_enabled: bool = True
    ble_enabled: bool = True


def _float_field(fields: Mapping[str, Any], key: str, *, default: float | None = None) -> float:
    if key not in fields:
        if default is not None:
            return default
        raise ValueError(f"missing required field {key!r}")
    value = fields[key]
    if isinstance(value, (int, float)):
        return float(value)
    if hasattr(value, "number_value"):
        return float(value.number_value)
    raise ValueError(f"{key!r} must be a number")


def _optional_float_field(fields: Mapping[str, Any], key: str) -> float | None:
    if key not in fields:
        return None
    return _float_field(fields, key)


def _string_field(fields: Mapping[str, Any], key: str) -> str:
    if key not in fields:
        raise ValueError(f"missing required field {key!r}")
    value = fields[key]
    if isinstance(value, str):
        return value
    if hasattr(value, "string_value"):
        return value.string_value
    raise ValueError(f"{key!r} must be a string")


def _struct_fields(value: Any) -> dict[str, Any]:
    if isinstance(value, dict):
        return value
    if hasattr(value, "struct_value"):
        return dict(value.struct_value.fields)
    raise ValueError("expected a struct/object")


def _config_float(
    raw: Mapping[str, Any],
    floor: Mapping[str, Any],
    key: str,
    *,
    default: float = 0.0,
) -> float:
    if key in raw:
        return float(raw[key])
    if key in floor:
        return float(floor[key])
    return default


def _parse_access_point_item(
    item: Any,
    *,
    default_z_m: float,
) -> ConfiguredAccessPoint:
    fields = _struct_fields(item)
    name = _string_field(fields, "name")
    bssid = normalize_mac(_string_field(fields, "bssid"))
    z_override = _optional_float_field(fields, "z_m")
    return ConfiguredAccessPoint(
        name=name,
        x_m=_float_field(fields, "x_m"),
        y_m=_float_field(fields, "y_m"),
        z_m=z_override if z_override is not None else default_z_m,
        bssid=bssid,
    )


def _parse_ble_beacon_item(item: Any) -> BleBeacon:
    fields = _struct_fields(item)
    name = _string_field(fields, "name")
    mac = _string_field(fields, "mac_address").lower().strip()
    z_m = _float_field(fields, "z_m", default=1.0)
    tx_power = _float_field(fields, "tx_power_dbm", default=-59.0)
    path_loss = _float_field(fields, "path_loss_n", default=2.5)
    return BleBeacon(
        name=name,
        x_m=_float_field(fields, "x_m"),
        y_m=_float_field(fields, "y_m"),
        z_m=z_m,
        mac_address=mac,
        tx_power_dbm=tx_power,
        path_loss_n=path_loss,
    )


def parse_config_dict(raw: dict[str, Any]) -> LocatorConfig:
    """Parse module config from a plain JSON object (local testing / export).

    At least one signal source must be configured:

    * WiFi: provide ``access_points``, ``scan_ssid``, and ``scan_count``.
    * BLE:  provide ``ble_beacons``.
    * Both sources may be configured simultaneously.

    Use ``wifi_enabled: false`` or ``ble_enabled: false`` to override the
    auto-detected enabled state without removing the config entries.
    """
    floor = raw.get("floor_plan") or {}
    x_origin_m = float(floor.get("x_origin_m", 0.0))
    y_origin_m = float(floor.get("y_origin_m", 0.0))
    device_z_m = _config_float(raw, floor, "device_z_m", default=0.0)
    access_point_z_m = _config_float(raw, floor, "access_point_z_m", default=0.0)
    width_m = float(floor["width_m"]) if "width_m" in floor else None
    height_m = float(floor["height_m"]) if "height_m" in floor else None
    if width_m is not None and width_m <= 0:
        raise ValueError("floor_plan.width_m must be > 0")
    if height_m is not None and height_m <= 0:
        raise ValueError("floor_plan.height_m must be > 0")

    has_aps = bool(raw.get("access_points"))
    has_ble = bool(raw.get("ble_beacons"))
    if not has_aps and not has_ble:
        raise ValueError(
            "at least one signal source must be configured: "
            "provide 'access_points' for WiFi, 'ble_beacons' for BLE, or both"
        )

    # WiFi scanning fields are required only when access_points are present
    scan_ssid = ""
    scan_count = 1
    aps: tuple[ConfiguredAccessPoint, ...] = ()
    if has_aps:
        if "scan_ssid" not in raw:
            raise ValueError("scan_ssid is required when access_points are configured")
        if "scan_count" not in raw:
            raise ValueError("scan_count is required when access_points are configured")
        scan_ssid = str(raw["scan_ssid"])
        scan_count = int(raw["scan_count"])
        if scan_count < 1:
            raise ValueError("scan_count must be >= 1")
        aps = tuple(
            _parse_access_point_item(ap, default_z_m=access_point_z_m)
            for ap in raw["access_points"]
        )
        if len(aps) < 1:
            raise ValueError("access_points must contain at least one AP")
    else:
        # BLE-only: scan_ssid/scan_count still accepted if provided (ignored)
        scan_ssid = str(raw.get("scan_ssid", ""))
        scan_count = int(raw.get("scan_count", 1))

    ble_beacons: tuple[BleBeacon, ...] = ()
    if has_ble:
        ble_beacons = tuple(_parse_ble_beacon_item(b) for b in raw["ble_beacons"])
    ble_min_rssi_dbm = float(raw.get("ble_min_rssi_dbm", -90.0))

    wifi_enabled = bool(raw.get("wifi_enabled", True))
    ble_enabled = bool(raw.get("ble_enabled", True))

    return LocatorConfig(
        scan_ssid=scan_ssid,
        scan_count=scan_count,
        x_origin_m=x_origin_m,
        y_origin_m=y_origin_m,
        device_z_m=device_z_m,
        access_point_z_m=access_point_z_m,
        access_points=aps,
        width_m=width_m,
        height_m=height_m,
        ble_beacons=ble_beacons,
        ble_min_rssi_dbm=ble_min_rssi_dbm,
        wifi_enabled=wifi_enabled,
        ble_enabled=ble_enabled,
    )


def parse_component_config(attributes: Any) -> LocatorConfig:
    """Parse config from Viam ComponentConfig.attributes (Struct or dict)."""
    if isinstance(attributes, dict):
        return parse_config_dict(attributes)

    try:
        from google.protobuf.json_format import MessageToDict
        from google.protobuf.struct_pb2 import Struct
    except ImportError as exc:
        raise ImportError(
            "google.protobuf is required for Viam ComponentConfig parsing; "
            "install the module with ./setup.sh or pip install viam-sdk"
        ) from exc

    if isinstance(attributes, Struct):
        return parse_config_dict(MessageToDict(attributes))
    if hasattr(attributes, "fields"):
        return parse_config_dict(MessageToDict(attributes))
    return parse_config_dict(dict(attributes))


def load_config_file(path: Path | str) -> LocatorConfig:
    raw = json.loads(Path(path).read_text())
    return parse_config_dict(raw)


def registry_from_config(config: LocatorConfig) -> ApRegistry:
    """Build an ApRegistry used by triangulation from module config."""
    aps = tuple(
        AccessPoint(
            ap_name=ap.name,
            x_m=ap.x_m,
            y_m=ap.y_m,
            bssid=ap.bssid,
            z_m=ap.z_m,
        )
        for ap in config.access_points
    )
    return ApRegistry(scan_ssid=config.scan_ssid, access_points=aps)
