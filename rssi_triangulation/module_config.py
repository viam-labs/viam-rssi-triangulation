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
    bssid: str


@dataclass(frozen=True)
class LocatorConfig:
    scan_ssid: str
    scan_count: int
    x_origin_m: float
    y_origin_m: float
    access_points: tuple[ConfiguredAccessPoint, ...]


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


def _parse_access_point_item(item: Any) -> ConfiguredAccessPoint:
    fields = _struct_fields(item)
    name = _string_field(fields, "name")
    bssid = normalize_mac(_string_field(fields, "bssid"))
    return ConfiguredAccessPoint(
        name=name,
        x_m=_float_field(fields, "x_m"),
        y_m=_float_field(fields, "y_m"),
        bssid=bssid,
    )


def parse_config_dict(raw: dict[str, Any]) -> LocatorConfig:
    """Parse module config from a plain JSON object (local testing / export)."""
    if "scan_ssid" not in raw:
        raise ValueError("scan_ssid is required")
    if "scan_count" not in raw:
        raise ValueError("scan_count is required")
    if "access_points" not in raw:
        raise ValueError("access_points is required")

    scan_count = int(raw["scan_count"])
    if scan_count < 1:
        raise ValueError("scan_count must be >= 1")

    floor = raw.get("floor_plan") or {}
    x_origin_m = float(floor.get("x_origin_m", 0.0))
    y_origin_m = float(floor.get("y_origin_m", 0.0))

    aps = tuple(_parse_access_point_item(ap) for ap in raw["access_points"])
    if len(aps) < 1:
        raise ValueError("access_points must contain at least one AP")

    return LocatorConfig(
        scan_ssid=str(raw["scan_ssid"]),
        scan_count=scan_count,
        x_origin_m=x_origin_m,
        y_origin_m=y_origin_m,
        access_points=aps,
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
        AccessPoint(ap_name=ap.name, x_m=ap.x_m, y_m=ap.y_m, bssid=ap.bssid)
        for ap in config.access_points
    )
    return ApRegistry(scan_ssid=config.scan_ssid, access_points=aps)
