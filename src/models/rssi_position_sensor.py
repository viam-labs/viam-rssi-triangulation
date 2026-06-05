"""Viam sensor: WiFi RSSI floor position."""

from __future__ import annotations

import asyncio
from pathlib import Path
from typing import Any, ClassVar, Mapping, Sequence, Tuple

from typing_extensions import Self

from viam.components.sensor import Sensor
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.resource.types import Model, ModelFamily
from viam.utils import SensorReading, ValueTypes

from rssi_triangulation.fingerprint import FingerprintStore
from rssi_triangulation.fingerprint_commands import execute_fingerprint_command
from rssi_triangulation.locate import (
    PositionReading,
    build_readings_dict,
    locate_position,
    match_readings_to_aps,
    smooth_position,
)
from rssi_triangulation.module_config import (
    LocatorConfig,
    parse_component_config,
    registry_from_config,
)

MODEL: ClassVar[Model] = Model(
    ModelFamily("viam-labs", "rssi-triangulation"),
    "wifi-position",
)


def _command_dict(command: Mapping[str, ValueTypes]) -> dict[str, Any]:
    """Normalize Viam do_command payload to plain Python values."""
    out: dict[str, Any] = {}
    for key, value in command.items():
        if isinstance(value, str):
            out[key] = value
        elif isinstance(value, bool):
            out[key] = value
        elif isinstance(value, (int, float)):
            out[key] = value
        elif hasattr(value, "string_value"):
            out[key] = value.string_value
        elif hasattr(value, "number_value"):
            out[key] = value.number_value
        elif hasattr(value, "bool_value"):
            out[key] = value.bool_value
        else:
            out[key] = value
    return out


class RssiPositionSensor(Sensor, EasyResource):
    MODEL = MODEL

    _config: LocatorConfig
    _interface: str | None
    _backend: str | None
    _scan_delay_s: float
    _blocking_scan: bool
    _strict_mac: bool
    _method: str
    _min_anchors: int
    _max_rssi_delta_db: float | None
    _min_rssi_dbm: float
    _min_samples_per_ap: int | None
    _tx_power_dbm: float
    _path_loss_n: float
    _weight_temperature: float
    _fingerprint_db_path: Path
    _fingerprint_k: int
    _fingerprint_min_common_aps: int
    _fingerprint_min_common_fraction: float
    _fingerprint_max_rms_db: float | None
    _fingerprint_max_blend: float
    _fingerprint_fallback: bool
    _fast_scan: bool
    _smoothing_alpha: float
    _max_position_step_m: float | None
    _last_position: PositionReading | None
    _fingerprint_store: FingerprintStore | None
    _device_z_m: float

    @classmethod
    def new(
        cls,
        config: ComponentConfig,
        dependencies: Mapping[ResourceName, ResourceBase],
    ) -> Self:
        sensor = super().new(config, dependencies)
        fields = config.attributes.fields
        sensor._config = parse_component_config(config.attributes)
        sensor._interface = (
            fields["interface"].string_value if "interface" in fields else None
        )
        sensor._backend = fields["backend"].string_value if "backend" in fields else None
        sensor._scan_delay_s = (
            fields["scan_delay_s"].number_value if "scan_delay_s" in fields else 0.15
        )
        sensor._blocking_scan = (
            fields["blocking_scan"].bool_value if "blocking_scan" in fields else False
        )
        sensor._strict_mac = (
            fields["strict_mac"].bool_value if "strict_mac" in fields else True
        )
        sensor._method = (
            fields["method"].string_value
            if "method" in fields
            else "hybrid"
        )
        sensor._min_anchors = (
            int(fields["min_anchors"].number_value) if "min_anchors" in fields else 3
        )
        sensor._max_rssi_delta_db = (
            fields["max_rssi_delta_db"].number_value
            if "max_rssi_delta_db" in fields
            else 35.0
        )
        sensor._min_rssi_dbm = (
            fields["min_rssi_dbm"].number_value
            if "min_rssi_dbm" in fields
            else -90.0
        )
        sensor._min_samples_per_ap = (
            int(fields["min_samples_per_ap"].number_value)
            if "min_samples_per_ap" in fields
            else None
        )
        sensor._tx_power_dbm = (
            fields["tx_power_dbm"].number_value if "tx_power_dbm" in fields else -40.0
        )
        sensor._path_loss_n = (
            fields["path_loss_n"].number_value if "path_loss_n" in fields else 2.5
        )
        sensor._weight_temperature = (
            fields["weight_temperature"].number_value
            if "weight_temperature" in fields
            else 2.0
        )
        db_path = (
            fields["fingerprint_db_path"].string_value
            if "fingerprint_db_path" in fields
            else "fingerprints.sqlite"
        )
        sensor._fingerprint_db_path = Path(db_path)
        sensor._fingerprint_k = (
            int(fields["fingerprint_k"].number_value) if "fingerprint_k" in fields else 1
        )
        sensor._fingerprint_min_common_aps = (
            int(fields["fingerprint_min_common_aps"].number_value)
            if "fingerprint_min_common_aps" in fields
            else 3
        )
        sensor._fingerprint_min_common_fraction = (
            fields["fingerprint_min_common_fraction"].number_value
            if "fingerprint_min_common_fraction" in fields
            else 0.5
        )
        sensor._fingerprint_max_rms_db = (
            fields["fingerprint_max_rms_db"].number_value
            if "fingerprint_max_rms_db" in fields
            else 10.0
        )
        sensor._fingerprint_max_blend = (
            fields["fingerprint_max_blend"].number_value
            if "fingerprint_max_blend" in fields
            else 0.5
        )
        sensor._fingerprint_fallback = (
            fields["fingerprint_fallback"].bool_value
            if "fingerprint_fallback" in fields
            else True
        )
        sensor._fast_scan = (
            fields["fast_scan"].bool_value if "fast_scan" in fields else True
        )
        if "thorough_scan" in fields and fields["thorough_scan"].bool_value:
            sensor._fast_scan = False
        sensor._smoothing_alpha = (
            fields["smoothing_alpha"].number_value
            if "smoothing_alpha" in fields
            else 1.0
        )
        sensor._max_position_step_m = (
            fields["max_position_step_m"].number_value
            if "max_position_step_m" in fields
            else 0.0
        )
        sensor._last_position = None
        sensor._fingerprint_store = None
        sensor._device_z_m = sensor._config.device_z_m
        return sensor

    def _get_fingerprint_store(self) -> FingerprintStore:
        if self._fingerprint_store is None:
            self._fingerprint_store = FingerprintStore(self._fingerprint_db_path)
        return self._fingerprint_store

    @classmethod
    def validate_config(
        cls, config: ComponentConfig
    ) -> Tuple[Sequence[str], Sequence[str]]:
        parse_component_config(config.attributes)
        return [], []

    async def get_readings(
        self,
        *,
        extra: Mapping[str, Any] | None = None,
        timeout: float | None = None,
        **kwargs,
    ) -> Mapping[str, SensorReading]:
        del extra, timeout, kwargs
        fp_store = (
            self._get_fingerprint_store()
            if self._method in ("fingerprint", "hybrid")
            else None
        )
        raw_xy, backend, readings, scans, method_used, fp_match = await asyncio.to_thread(
            locate_position,
            self._config,
            device_z_m=self._device_z_m,
            interface=self._interface,
            backend=self._backend,
            scan_delay_s=self._scan_delay_s,
            blocking=self._blocking_scan,
            strict_mac=self._strict_mac,
            method=self._method,
            min_anchors=self._min_anchors,
            max_rssi_delta_db=self._max_rssi_delta_db,
            min_rssi_dbm=self._min_rssi_dbm,
            min_samples_per_ap=self._min_samples_per_ap,
            tx_power_dbm=self._tx_power_dbm,
            path_loss_n=self._path_loss_n,
            weight_temperature=self._weight_temperature,
            fingerprint_store=fp_store,
            fingerprint_k=self._fingerprint_k,
            fingerprint_min_common_aps=self._fingerprint_min_common_aps,
            fingerprint_min_common_fraction=self._fingerprint_min_common_fraction,
            fingerprint_max_rms_db=self._fingerprint_max_rms_db,
            fingerprint_max_blend=self._fingerprint_max_blend,
            fingerprint_fallback=self._fingerprint_fallback,
            fast_scan=self._fast_scan,
        )
        raw_position = raw_xy
        position = smooth_position(
            self._last_position,
            raw_position,
            alpha=self._smoothing_alpha,
            max_step_m=(
                None if self._max_position_step_m <= 0 else self._max_position_step_m
            ),
        )
        self._last_position = position
        fp_detail = ""
        if fp_match is not None:
            blend = (
                f" blend={fp_match.blend_weight:.2f}"
                if fp_match.blend_weight > 0
                else ""
            )
            fp_detail = (
                f", fp={fp_match.label} rms={fp_match.distance_db:.1f}dB"
                f"{blend} neighbors={','.join(fp_match.neighbors)}"
            )
        self.logger.debug(
            "position (%.2f, %.2f, %.2f) m raw (%.2f, %.2f) via %s (%s%s), %d BSSIDs on SSID, %d scans",
            position.x_m,
            position.y_m,
            position.z_m,
            raw_position.x_m,
            raw_position.y_m,
            backend,
            method_used,
            fp_detail,
            len(readings),
            scans,
        )
        matched = match_readings_to_aps(
            readings,
            registry_from_config(self._config),
            strict_mac=self._strict_mac,
            min_sample_count=self._min_samples_per_ap or (2 if scans >= 3 else 1),
        )
        return build_readings_dict(position, matched, self._config)

    async def do_command(
        self,
        command: Mapping[str, ValueTypes],
        *,
        timeout: float | None = None,
        **kwargs,
    ) -> Mapping[str, ValueTypes]:
        del timeout, kwargs
        cmd = _command_dict(command)
        if cmd.get("command") == "set_device_z_m":
            if "z_m" not in cmd:
                raise ValueError('set_device_z_m requires "z_m"')
            self._device_z_m = float(cmd["z_m"])
            return {
                "ok": True,
                "command": "set_device_z_m",
                "z_m": self._device_z_m,
            }
        result = await asyncio.to_thread(
            execute_fingerprint_command,
            cmd,
            config=self._config,
            db=self._get_fingerprint_store(),
            interface=self._interface,
            backend=self._backend,
            scan_delay_s=self._scan_delay_s,
            blocking=self._blocking_scan,
            strict_mac=self._strict_mac,
            min_samples_per_ap=self._min_samples_per_ap,
            device_z_m=self._device_z_m,
        )
        return result
