"""Estimate client position from RSSI and known AP floor-plan coordinates."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .registry import AccessPoint, ApRegistry


@dataclass(frozen=True)
class Anchor:
    ap_name: str
    x_m: float
    y_m: float
    rssi_dbm: float
    z_m: float = 0.0


@dataclass(frozen=True)
class PositionEstimate:
    x_m: float
    y_m: float
    unit: str
    method: str
    anchor_count: int
    anchors_used: tuple[str, ...]
    residual_rmse_m: float | None = None

    def as_dict(self) -> dict:
        return {
            "x_m": self.x_m,
            "y_m": self.y_m,
            "unit": self.unit,
            "method": self.method,
            "anchor_count": self.anchor_count,
            "anchors_used": list(self.anchors_used),
            "residual_rmse_m": self.residual_rmse_m,
        }


def anchors_from_readings(
    registry: ApRegistry,
    readings: list[tuple[str, float, float | None]],
) -> list[Anchor]:
    """
    Build one anchor per AP name, keeping the strongest RSSI when duplicates exist.

    readings: (ap_name, rssi_dbm, frequency_mhz optional)
    """
    positions = registry.name_to_position
    best: dict[str, Anchor] = {}
    for name, rssi, _freq in readings:
        pos = positions.get(name)
        if pos is None:
            continue
        x, y, z = pos
        prev = best.get(name)
        if prev is None or rssi > prev.rssi_dbm:
            best[name] = Anchor(ap_name=name, x_m=x, y_m=y, rssi_dbm=rssi, z_m=z)
    return list(best.values())


def delta_soft_weight(
    delta_db: float,
    max_delta_db: float,
    *,
    softness_db: float | None = None,
) -> float:
    """Smooth weight in [0, 1] for an anchor's dB gap below the strongest signal.

    Full weight below ``max_delta_db - softness_db``; cosine ease to zero above
    ``max_delta_db + softness_db``. Avoids the position cliffs caused by a hard
    in/out cutoff when an AP hovers near ``max_rssi_delta_db``.
    """
    if max_delta_db <= 0:
        return 0.0
    if softness_db is None:
        softness_db = max(6.0, 0.35 * max_delta_db)
    low = max_delta_db - softness_db
    high = max_delta_db + softness_db
    if delta_db <= low:
        return 1.0
    if delta_db >= high:
        return 0.0
    t = (delta_db - low) / (high - low)
    return 0.5 * (1.0 + math.cos(math.pi * t))


def filter_anchors(
    anchors: list[Anchor],
    *,
    max_delta_db: float | None = 20.0,
    min_rssi_dbm: float = -82.0,
) -> list[Anchor]:
    """
    Drop anchors below ``min_rssi_dbm``.

    Relative filtering vs the strongest AP is applied as a soft weight multiplier
    (see ``combined_anchor_weights``) rather than a hard cutoff, so anchors near
    ``max_delta_db`` fade out gradually instead of toggling the anchor set.

    ``max_delta_db`` is accepted for API compatibility but does not remove
    anchors; pass ``None`` to disable the soft relative weighting as well.
    """
    del max_delta_db
    return [a for a in anchors if a.rssi_dbm >= min_rssi_dbm]


def combined_anchor_weights(
    anchors: list[Anchor],
    *,
    weight_temperature: float = 2.0,
    max_delta_db: float | None = 20.0,
) -> list[float]:
    """RSSI weights multiplied by soft relative-signal weights."""
    if not anchors:
        return []
    rssi_weights = _weights_from_rssi(anchors, weight_temperature)
    if max_delta_db is None:
        return rssi_weights
    strongest = max(a.rssi_dbm for a in anchors)
    return [
        rw * delta_soft_weight(strongest - a.rssi_dbm, max_delta_db)
        for a, rw in zip(anchors, rssi_weights)
    ]


def effective_anchors(
    anchors: list[Anchor],
    weights: list[float],
    *,
    min_fraction: float = 0.02,
) -> list[Anchor]:
    """Anchors whose combined weight is at least ``min_fraction`` of the peak."""
    if not anchors or not weights:
        return []
    peak = max(weights)
    if peak <= 0:
        return []
    threshold = peak * min_fraction
    return [a for a, w in zip(anchors, weights) if w >= threshold]


def distance_3d_m(
    x: float,
    y: float,
    device_z_m: float,
    anchor: Anchor,
) -> float:
    """Slant range from (x, y, device_z) to an anchor in 3D."""
    dz = device_z_m - anchor.z_m
    return math.sqrt((x - anchor.x_m) ** 2 + (y - anchor.y_m) ** 2 + dz * dz)


def has_vertical_geometry(
    device_z_m: float,
    anchors: list[Anchor],
    *,
    threshold_m: float = 0.05,
) -> bool:
    """True when AP height and device height differ enough to affect range."""
    return any(abs(device_z_m - a.z_m) > threshold_m for a in anchors)


def _weights_from_rssi(anchors: list[Anchor], weight_temperature: float = 2.0) -> list[float]:
    """
    Stronger RSSI (less negative) gets higher weight.

    `weight_temperature` softens the falloff: 1.0 weights by linear received power
    (winner-take-all, jumpy); higher values flatten weights so several anchors
    contribute and the centroid does not swing when the strongest AP changes.
    """
    if not anchors:
        return []
    temp = weight_temperature if weight_temperature > 0 else 1.0
    max_rssi = max(a.rssi_dbm for a in anchors)
    return [10 ** ((a.rssi_dbm - max_rssi) / (10 * temp)) for a in anchors]


def rssi_to_distance_m(
    rssi_dbm: float,
    *,
    tx_power_dbm: float = -40.0,
    path_loss_n: float = 2.5,
) -> float:
    """Log-distance path loss: d meters from RSSI."""
    return 10 ** ((tx_power_dbm - rssi_dbm) / (10 * path_loss_n))


def refine_position_path_loss_3d(
    x: float,
    y: float,
    anchors: list[Anchor],
    *,
    device_z_m: float,
    tx_power_dbm: float = -40.0,
    path_loss_n: float = 2.5,
    weight_temperature: float = 2.0,
    weights: list[float] | None = None,
    max_iter: int = 80,
    learning_rate: float = 0.15,
) -> tuple[float, float]:
    """
    Refine (x, y) by minimizing (3D geometric distance - path-loss distance)^2.

    When the device is directly under a ceiling AP, slant range is mostly vertical
    so horizontal position stays at the AP's x/y instead of being pushed outward.
    """
    if len(anchors) < 2:
        return x, y

    if weights is None:
        weights = _weights_from_rssi(anchors, weight_temperature)
    wsum = sum(weights)
    if wsum <= 0:
        return x, y

    for _ in range(max_iter):
        gx, gy = 0.0, 0.0
        for a, w in zip(anchors, weights):
            if w <= 0:
                continue
            d_model = rssi_to_distance_m(
                a.rssi_dbm, tx_power_dbm=tx_power_dbm, path_loss_n=path_loss_n
            )
            dx = x - a.x_m
            dy = y - a.y_m
            dist = distance_3d_m(x, y, device_z_m, a)
            if dist < 0.01:
                continue
            err = dist - d_model
            gx += w * err * (dx / dist)
            gy += w * err * (dy / dist)
        x -= learning_rate * gx / wsum
        y -= learning_rate * gy / wsum
    return x, y


def estimate_weighted_centroid(
    anchors: list[Anchor],
    unit: str = "m",
    *,
    weight_temperature: float = 2.0,
    device_z_m: float = 0.0,
    tx_power_dbm: float = -40.0,
    path_loss_n: float = 2.5,
    refine_3d: bool = True,
    weights: list[float] | None = None,
) -> PositionEstimate | None:
    if not anchors:
        return None
    if len(anchors) == 1:
        a = anchors[0]
        return PositionEstimate(
            x_m=a.x_m,
            y_m=a.y_m,
            unit=unit,
            method="nearest_ap",
            anchor_count=1,
            anchors_used=(a.ap_name,),
            residual_rmse_m=0.0,
        )

    if weights is None:
        weights = _weights_from_rssi(anchors, weight_temperature)
    wsum = sum(weights)
    if wsum <= 0:
        return None
    x = sum(w * a.x_m for a, w in zip(anchors, weights)) / wsum
    y = sum(w * a.y_m for a, w in zip(anchors, weights)) / wsum
    method = "weighted_centroid"
    if refine_3d and has_vertical_geometry(device_z_m, anchors):
        dominant = max(weights)
        if dominant / wsum >= 0.7:
            lead = anchors[weights.index(dominant)]
            d_geo = distance_3d_m(lead.x_m, lead.y_m, device_z_m, lead)
            d_model = rssi_to_distance_m(
                lead.rssi_dbm,
                tx_power_dbm=tx_power_dbm,
                path_loss_n=path_loss_n,
            )
            if abs(d_geo - d_model) <= max(0.5, 0.2 * d_model):
                x, y = lead.x_m, lead.y_m
                method = "weighted_centroid_3d"
            else:
                x, y = refine_position_path_loss_3d(
                    lead.x_m,
                    lead.y_m,
                    anchors,
                    device_z_m=device_z_m,
                    tx_power_dbm=tx_power_dbm,
                    path_loss_n=path_loss_n,
                    weights=weights,
                    max_iter=40,
                )
                method = "weighted_centroid_3d"
        else:
            x, y = refine_position_path_loss_3d(
                x,
                y,
                anchors,
                device_z_m=device_z_m,
                tx_power_dbm=tx_power_dbm,
                path_loss_n=path_loss_n,
                weights=weights,
                max_iter=40,
            )
            method = "weighted_centroid_3d"
    rmse = _weighted_rmse(anchors, x, y, weights, device_z_m=device_z_m)
    used = effective_anchors(anchors, weights)
    return PositionEstimate(
        x_m=x,
        y_m=y,
        unit=unit,
        method=method,
        anchor_count=len(used),
        anchors_used=tuple(a.ap_name for a in used),
        residual_rmse_m=rmse,
    )


def clamp_to_bounds(
    x: float,
    y: float,
    aps: list[Anchor] | tuple,
    *,
    margin_m: float,
) -> tuple[float, float, bool]:
    """Clamp (x, y) to the APs' bounding box expanded by ``margin_m``.

    Path-loss refinement can extrapolate far outside the deployment when the
    model and geometry disagree (e.g. miscalibrated tx_power / n); positions
    far beyond every AP are never physically meaningful.
    """
    xs = [a.x_m for a in aps]
    ys = [a.y_m for a in aps]
    cx = min(max(x, min(xs) - margin_m), max(xs) + margin_m)
    cy = min(max(y, min(ys) - margin_m), max(ys) + margin_m)
    return cx, cy, (cx != x or cy != y)


def estimate_path_loss_ls(
    anchors: list[Anchor],
    *,
    unit: str = "m",
    device_z_m: float = 0.0,
    tx_power_dbm: float = -40.0,
    path_loss_n: float = 2.5,
    weight_temperature: float = 2.0,
    weights: list[float] | None = None,
    max_iter: int = 80,
    learning_rate: float = 0.15,
) -> PositionEstimate | None:
    """
    Refine position by minimizing (3D distance - path-loss distance)^2.
    Starts from weighted centroid; requires calibration (tx_power, n).
    """
    if weights is None:
        weights = _weights_from_rssi(anchors, weight_temperature)
    seed = estimate_weighted_centroid(
        anchors,
        unit=unit,
        device_z_m=device_z_m,
        tx_power_dbm=tx_power_dbm,
        path_loss_n=path_loss_n,
        refine_3d=False,
        weights=weights,
    )
    if seed is None:
        return None
    if len(anchors) < 2:
        return seed

    x, y = refine_position_path_loss_3d(
        seed.x_m,
        seed.y_m,
        anchors,
        device_z_m=device_z_m,
        tx_power_dbm=tx_power_dbm,
        path_loss_n=path_loss_n,
        weights=weights,
        max_iter=max_iter,
        learning_rate=learning_rate,
    )
    rmse = _path_loss_rmse(
        anchors,
        x,
        y,
        device_z_m=device_z_m,
        tx_power_dbm=tx_power_dbm,
        path_loss_n=path_loss_n,
    )
    return PositionEstimate(
        x_m=x,
        y_m=y,
        unit=unit,
        method="path_loss_ls",
        anchor_count=len(anchors),
        anchors_used=tuple(a.ap_name for a in anchors),
        residual_rmse_m=rmse,
    )


def estimate_position(
    registry: ApRegistry,
    readings: list[tuple[str, float, float | None]],
    *,
    method: str = "weighted_centroid",
    min_anchors: int = 3,
    device_z_m: float = 0.0,
    tx_power_dbm: float = -40.0,
    path_loss_n: float = 2.5,
    max_rssi_delta_db: float | None = 20.0,
    min_rssi_dbm: float = -82.0,
    weight_temperature: float = 2.0,
    clamp_margin_m: float | None = 5.0,
) -> PositionEstimate | None:
    anchors = filter_anchors(
        anchors_from_readings(registry, readings),
        min_rssi_dbm=min_rssi_dbm,
    )
    weights = combined_anchor_weights(
        anchors,
        weight_temperature=weight_temperature,
        max_delta_db=max_rssi_delta_db,
    )
    if sum(1 for w in weights if w > 0) < min_anchors:
        return None
    unit = registry.unit
    if method == "path_loss":
        estimate = estimate_path_loss_ls(
            anchors,
            unit=unit,
            device_z_m=device_z_m,
            tx_power_dbm=tx_power_dbm,
            path_loss_n=path_loss_n,
            weight_temperature=weight_temperature,
            weights=weights,
        )
    else:
        estimate = estimate_weighted_centroid(
            anchors,
            unit=unit,
            weight_temperature=weight_temperature,
            device_z_m=device_z_m,
            tx_power_dbm=tx_power_dbm,
            path_loss_n=path_loss_n,
            weights=weights,
        )
    if estimate is None or clamp_margin_m is None:
        return estimate
    # Bound against all configured APs (not just heard anchors) so being near
    # the floor edge by an unheard AP is not penalized.
    x, y, clamped = clamp_to_bounds(
        estimate.x_m,
        estimate.y_m,
        registry.access_points,
        margin_m=clamp_margin_m,
    )
    if not clamped:
        return estimate
    return PositionEstimate(
        x_m=x,
        y_m=y,
        unit=estimate.unit,
        method=estimate.method + "_clamped",
        anchor_count=estimate.anchor_count,
        anchors_used=estimate.anchors_used,
        residual_rmse_m=estimate.residual_rmse_m,
    )


def _weighted_rmse(
    anchors: list[Anchor],
    x: float,
    y: float,
    weights: list[float],
    *,
    device_z_m: float = 0.0,
) -> float:
    wsum = sum(weights)
    if wsum <= 0:
        return 0.0
    err_sq = sum(
        w * (distance_3d_m(x, y, device_z_m, a) ** 2)
        for a, w in zip(anchors, weights)
    )
    return math.sqrt(err_sq / wsum)


def _path_loss_rmse(
    anchors: list[Anchor],
    x: float,
    y: float,
    *,
    device_z_m: float,
    tx_power_dbm: float,
    path_loss_n: float,
) -> float:
    if not anchors:
        return 0.0
    errs = []
    for a in anchors:
        d_geo = distance_3d_m(x, y, device_z_m, a)
        d_model = rssi_to_distance_m(
            a.rssi_dbm, tx_power_dbm=tx_power_dbm, path_loss_n=path_loss_n
        )
        errs.append(d_geo - d_model)
    return math.sqrt(sum(e * e for e in errs) / len(errs))
