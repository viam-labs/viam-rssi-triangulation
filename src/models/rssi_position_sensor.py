"""Viam sensor: WiFi RSSI floor position."""

from __future__ import annotations

import asyncio
import math
from time import monotonic
from pathlib import Path
from typing import Any, ClassVar, Mapping, Sequence, Tuple, cast

from typing_extensions import Self

from viam.components.base import Base
from viam.components.movement_sensor import MovementSensor
from viam.components.sensor import Sensor
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.resource.types import Model, ModelFamily
from viam.services.slam import SLAMClient
from viam.utils import SensorReading, ValueTypes

from rssi_triangulation.fingerprint import FingerprintStore
from rssi_triangulation.fingerprint_commands import execute_fingerprint_command
from rssi_triangulation.fusion import (
    MotionDelta,
    PositionFilter,
    measurement_var_from_fix,
    slam_pose_delta,
)
from rssi_triangulation.locate import (
    PositionReading,
    build_readings_dict,
    estimate_from_matched,
    locate_position,
    match_readings_to_aps,
    smooth_position,
)
from rssi_triangulation.scanner import BackgroundScanner
from rssi_triangulation.module_config import (
    LocatorConfig,
    parse_component_config,
    registry_from_config,
)

MODEL: ClassVar[Model] = Model(
    ModelFamily("viam-labs", "rssi-triangulation"),
    "wifi-position",
)


def _optional_name(fields: Mapping[str, Any], key: str) -> str | None:
    """Read an optional resource-name string from config fields."""
    if key not in fields:
        return None
    value = fields[key].string_value
    return value or None


def _motion_source_names(fields: Mapping[str, Any]) -> dict[str, str]:
    """Configured motion-source resource names, keyed by kind (omitted when unset)."""
    names: dict[str, str] = {}
    for key in ("movement_sensor", "base", "slam"):
        name = _optional_name(fields, key)
        if name:
            names[key] = name
    return names


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
    _fast_scan: bool
    _smoothing_alpha: float
    _max_position_step_m: float | None
    _last_position: PositionReading | None
    _fingerprint_store: FingerprintStore | None
    _device_z_m: float
    _movement_sensor: MovementSensor | None
    _base: Base | None
    _slam: SLAMClient | None
    _fusion_enabled: bool
    _position_filter: PositionFilter | None
    _fusion_measurement_noise_m: float
    _fusion_max_innovation_m: float
    _base_moving_speed_mps: float
    _slam_yaw_offset_deg: float
    _slam_scale: float
    _last_motion_time: float | None
    _last_slam_xy_mm: tuple[float, float] | None
    _scanner: BackgroundScanner | None

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
        sensor._strict_mac = (
            fields["strict_mac"].bool_value if "strict_mac" in fields else True
        )
        scan_mode = (
            fields["scan_mode"].string_value if "scan_mode" in fields else "fast"
        )
        if scan_mode not in ("fast", "thorough", "blocking"):
            raise ValueError(
                f"scan_mode must be 'fast', 'thorough', or 'blocking', got {scan_mode!r}"
            )
        sensor._fast_scan = scan_mode == "fast"
        sensor._blocking_scan = scan_mode == "blocking"
        sensor._min_anchors = (
            int(fields["min_anchors"].number_value) if "min_anchors" in fields else 3
        )
        sensor._max_rssi_delta_db = (
            fields["max_rssi_delta_db"].number_value
            if "max_rssi_delta_db" in fields
            else 20.0
        )
        sensor._min_rssi_dbm = (
            fields["min_rssi_dbm"].number_value
            if "min_rssi_dbm" in fields
            else -82.0
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

        source_names = _motion_source_names(fields)
        sensor._movement_sensor = cast(
            "MovementSensor | None",
            cls._lookup_dependency(
                dependencies, MovementSensor, source_names.get("movement_sensor")
            ),
        )
        sensor._base = cast(
            "Base | None",
            cls._lookup_dependency(dependencies, Base, source_names.get("base")),
        )
        sensor._slam = cast(
            "SLAMClient | None",
            cls._lookup_dependency(dependencies, SLAMClient, source_names.get("slam")),
        )
        has_source = any(
            (sensor._movement_sensor, sensor._base, sensor._slam)
        )
        sensor._fusion_enabled = (
            fields["motion_fusion"].bool_value
            if "motion_fusion" in fields
            else has_source
        )
        sensor._fusion_measurement_noise_m = (
            fields["fusion_measurement_noise_m"].number_value
            if "fusion_measurement_noise_m" in fields
            else 3.0
        )
        sensor._fusion_max_innovation_m = (
            fields["fusion_max_innovation_m"].number_value
            if "fusion_max_innovation_m" in fields
            else 8.0
        )
        sensor._base_moving_speed_mps = (
            fields["base_moving_speed_mps"].number_value
            if "base_moving_speed_mps" in fields
            else 0.5
        )
        sensor._slam_yaw_offset_deg = (
            fields["slam_yaw_offset_deg"].number_value
            if "slam_yaw_offset_deg" in fields
            else 0.0
        )
        sensor._slam_scale = (
            fields["slam_scale"].number_value if "slam_scale" in fields else 1.0
        )
        if sensor._fusion_enabled and has_source:
            sensor._position_filter = PositionFilter(
                process_noise_m=(
                    fields["fusion_process_noise_m"].number_value
                    if "fusion_process_noise_m" in fields
                    else 0.5
                ),
                measurement_noise_m=sensor._fusion_measurement_noise_m,
                max_innovation_m=sensor._fusion_max_innovation_m,
                speed_scale=(
                    fields["fusion_speed_scale"].number_value
                    if "fusion_speed_scale" in fields
                    else 1.0
                ),
            )
        else:
            sensor._position_filter = None
        sensor._last_motion_time = None
        sensor._last_slam_xy_mm = None

        # Default on: the sensor is polled repeatedly on a robot, which is the
        # continuous case background scanning was built for.
        background_scan = (
            fields["background_scan"].bool_value
            if "background_scan" in fields
            else True
        )
        if background_scan:
            sensor._scanner = BackgroundScanner(
                interface=sensor._interface,
                network=sensor._config.scan_ssid,
                backend=sensor._backend,
                fast=sensor._fast_scan,
                blocking=sensor._blocking_scan,
                interval_s=(
                    fields["background_scan_interval_s"].number_value
                    if "background_scan_interval_s" in fields
                    else 0.5
                ),
                window_s=(
                    fields["rssi_window_s"].number_value
                    if "rssi_window_s" in fields
                    else 8.0
                ),
                half_life_s=(
                    fields["rssi_half_life_s"].number_value
                    if "rssi_half_life_s" in fields
                    else 2.5
                ),
            )
            sensor._scanner.start()
        else:
            sensor._scanner = None
        return sensor

    @staticmethod
    def _lookup_dependency(
        dependencies: Mapping[ResourceName, ResourceBase],
        resource_cls: Any,
        name: str | None,
    ) -> ResourceBase | None:
        """Resolve an optional configured dependency by resource name."""
        if not name:
            return None
        resource_name = resource_cls.get_resource_name(name)
        resource = dependencies.get(resource_name)
        if resource is not None:
            return resource
        # Fall back to matching on the short name if the key differs.
        for key, dep in dependencies.items():
            if key.name == name:
                return dep
        return None

    def _get_fingerprint_store(self) -> FingerprintStore:
        if self._fingerprint_store is None:
            self._fingerprint_store = FingerprintStore(self._fingerprint_db_path)
        return self._fingerprint_store

    async def close(self) -> None:
        if self._scanner is not None:
            await asyncio.to_thread(self._scanner.stop)
            self._scanner = None
        await super().close()

    @classmethod
    def validate_config(
        cls, config: ComponentConfig
    ) -> Tuple[Sequence[str], Sequence[str]]:
        parse_component_config(config.attributes)
        # Motion sources are optional; declare configured ones so viam-server
        # injects them as dependencies for get_readings() fusion.
        optional_deps = list(_motion_source_names(config.attributes.fields).values())
        return [], optional_deps

    async def _read_motion(self, dt_s: float) -> MotionDelta:
        """Sample configured motion sources into a floor-frame ``MotionDelta``.

        Each source is best-effort: a failing read is logged and skipped rather
        than failing the position reading. SLAM (when present) supplies a
        directional pose delta; the movement sensor and base contribute speed /
        moving state used to adapt how aggressively the filter smooths.
        """
        sources: list[str] = []
        speed = 0.0
        dx = 0.0
        dy = 0.0
        has_direction = False
        is_moving = True

        if self._slam is not None:
            try:
                pose = await self._slam.get_position()
                curr = (pose.x, pose.y)
                if self._last_slam_xy_mm is not None:
                    dx, dy = slam_pose_delta(
                        self._last_slam_xy_mm,
                        curr,
                        yaw_offset_deg=self._slam_yaw_offset_deg,
                        scale=self._slam_scale,
                    )
                    has_direction = True
                    if dt_s > 0:
                        speed = max(speed, math.hypot(dx, dy) / dt_s)
                self._last_slam_xy_mm = curr
                sources.append("slam")
            except Exception as exc:  # best-effort: motion is optional
                self.logger.warning("slam motion read failed: %s", exc)

        if self._movement_sensor is not None:
            try:
                v = await self._movement_sensor.get_linear_velocity()
                speed = max(speed, math.hypot(v.x, v.y))
                sources.append("movement_sensor")
            except Exception as exc:  # best-effort: motion is optional
                self.logger.warning("movement_sensor read failed: %s", exc)

        if self._base is not None:
            try:
                moving = await self._base.is_moving()
                is_moving = moving
                if not moving:
                    speed = 0.0
                    dx = 0.0
                    dy = 0.0
                elif speed == 0.0:
                    speed = self._base_moving_speed_mps
                sources.append("base")
            except Exception as exc:  # best-effort: motion is optional
                self.logger.warning("base motion read failed: %s", exc)

        return MotionDelta(
            dx_m=dx,
            dy_m=dy,
            speed_mps=speed,
            is_moving=is_moving,
            has_direction=has_direction,
            sources=tuple(sources),
        )

    def _locate_from_buffer(self):
        """Estimate position from the background scanner's rolling buffer.

        Returns the same tuple shape as ``locate_position`` so the
        ``get_readings`` post-processing is shared between both paths.
        """
        scanner = self._scanner
        assert scanner is not None
        scanner.wait_for_data(timeout_s=5.0)
        aggregated, backend, scans = scanner.snapshot()
        if not aggregated:
            err = scanner.last_error
            raise RuntimeError(
                "background scanner has no recent WiFi samples"
                + (f" (last scan error: {err})" if err else "")
            )
        min_samples = self._min_samples_per_ap
        if min_samples is None:
            min_samples = 2 if scans >= 3 else 1
        matched = match_readings_to_aps(
            aggregated,
            registry_from_config(self._config),
            strict_mac=self._strict_mac,
            min_sample_count=min_samples,
        )
        position, method_used, fp_match = estimate_from_matched(
            self._config,
            matched,
            min_anchors=self._min_anchors,
            max_rssi_delta_db=self._max_rssi_delta_db,
            min_rssi_dbm=self._min_rssi_dbm,
            tx_power_dbm=self._tx_power_dbm,
            path_loss_n=self._path_loss_n,
            weight_temperature=self._weight_temperature,
            fingerprint_store=self._get_fingerprint_store(),
            fingerprint_k=self._fingerprint_k,
            fingerprint_min_common_aps=self._fingerprint_min_common_aps,
            fingerprint_min_common_fraction=self._fingerprint_min_common_fraction,
            fingerprint_max_rms_db=self._fingerprint_max_rms_db,
            fingerprint_max_blend=self._fingerprint_max_blend,
            device_z_m=self._device_z_m,
        )
        return position, backend, aggregated, scans, method_used, fp_match

    async def get_readings(
        self,
        *,
        extra: Mapping[str, Any] | None = None,
        timeout: float | None = None,
        **kwargs,
    ) -> Mapping[str, SensorReading]:
        del extra, timeout, kwargs
        if self._scanner is not None:
            (
                raw_xy,
                backend,
                readings,
                scans,
                method_used,
                fp_match,
            ) = await asyncio.to_thread(self._locate_from_buffer)
        else:
            raw_xy, backend, readings, scans, method_used, fp_match = await asyncio.to_thread(
                locate_position,
                self._config,
                device_z_m=self._device_z_m,
                interface=self._interface,
                backend=self._backend,
                scan_delay_s=self._scan_delay_s,
                blocking=self._blocking_scan,
                strict_mac=self._strict_mac,
                min_anchors=self._min_anchors,
                max_rssi_delta_db=self._max_rssi_delta_db,
                min_rssi_dbm=self._min_rssi_dbm,
                min_samples_per_ap=self._min_samples_per_ap,
                tx_power_dbm=self._tx_power_dbm,
                path_loss_n=self._path_loss_n,
                weight_temperature=self._weight_temperature,
                fingerprint_store=self._get_fingerprint_store(),
                fingerprint_k=self._fingerprint_k,
                fingerprint_min_common_aps=self._fingerprint_min_common_aps,
                fingerprint_min_common_fraction=self._fingerprint_min_common_fraction,
                fingerprint_max_rms_db=self._fingerprint_max_rms_db,
                fingerprint_max_blend=self._fingerprint_max_blend,
                fast_scan=self._fast_scan,
            )
        raw_position = raw_xy
        matched = match_readings_to_aps(
            readings,
            registry_from_config(self._config),
            strict_mac=self._strict_mac,
            min_sample_count=self._min_samples_per_ap or (2 if scans >= 3 else 1),
        )

        motion_detail = ""
        if self._position_filter is not None:
            now = monotonic()
            dt = (
                now - self._last_motion_time
                if self._last_motion_time is not None
                else 0.0
            )
            self._last_motion_time = now
            motion = await self._read_motion(dt)
            self._position_filter.predict(motion, dt)
            meas_var = measurement_var_from_fix(
                base_noise_m=self._fusion_measurement_noise_m,
                anchor_count=len(matched),
                fp_blend_weight=fp_match.blend_weight if fp_match is not None else 0.0,
            )
            accepted = self._position_filter.update(
                raw_position.x_m,
                raw_position.y_m,
                measurement_var_m2=meas_var,
                max_innovation_m=self._fusion_max_innovation_m,
            )
            fused = self._position_filter.position
            position = (
                PositionReading(x_m=fused[0], y_m=fused[1], z_m=raw_position.z_m)
                if fused is not None
                else raw_position
            )
            motion_detail = (
                f", motion[{'+'.join(motion.sources) or 'none'}]"
                f" v={motion.speed_mps:.2f}m/s{'' if accepted else ' gated'}"
            )
        else:
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
            "position (%.2f, %.2f, %.2f) m raw (%.2f, %.2f) via %s (%s%s%s), %d BSSIDs on SSID, %d scans",
            position.x_m,
            position.y_m,
            position.z_m,
            raw_position.x_m,
            raw_position.y_m,
            backend,
            method_used,
            fp_detail,
            motion_detail,
            len(readings),
            scans,
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
            fast_scan=self._fast_scan,
            current_tx_power_dbm=self._tx_power_dbm,
            current_path_loss_n=self._path_loss_n,
        )
        if cmd.get("command") == "calibrate_path_loss" and result.get("ok"):
            # Applied values live until restart; persist them in the component
            # config (tx_power_dbm / path_loss_n) to make them permanent.
            apply = bool(cmd.get("apply", False))
            if apply:
                self._tx_power_dbm = float(result["tx_power_dbm"])
                self._path_loss_n = float(result["path_loss_n"])
                self.logger.info(
                    "applied calibrated path loss: tx_power_dbm=%.2f path_loss_n=%.3f",
                    self._tx_power_dbm,
                    self._path_loss_n,
                )
            result["applied"] = apply
        return result
