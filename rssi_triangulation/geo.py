"""Floor-plan geometry helpers (slant range, trilateration)."""

from __future__ import annotations

import math
from typing import Mapping

from .module_config import LocatorConfig


def horizontal_radius_sq_from_slant(
    slant_m: float,
    *,
    device_z_m: float,
    ap_z_m: float,
) -> float:
    """Horizontal reach squared from a 3D slant range and vertical separation."""
    dz = device_z_m - ap_z_m
    ri_sq = slant_m * slant_m - dz * dz
    if ri_sq < 0.0:
        raise ValueError(
            f"slant range {slant_m:.2f} m is shorter than the {abs(dz):.2f} m "
            "vertical separation between the antenna and AP"
        )
    return ri_sq


def slant_range_m(
    x: float,
    y: float,
    *,
    device_z_m: float,
    ap_x: float,
    ap_y: float,
    ap_z: float,
) -> float:
    """3D distance from antenna (x, y, device_z) to AP (ap_x, ap_y, ap_z)."""
    dx = ap_x - x
    dy = ap_y - y
    dz = ap_z - device_z_m
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def infer_xy_from_slant_distances(
    ap_ranges: Mapping[str, tuple[float, float, float, float]],
    *,
    device_z_m: float,
    config: LocatorConfig,
    min_horizontal_m: float = 0.5,
) -> tuple[float, float] | None:
    """
    Infer antenna (x, y) in the reading frame from 2+ AP slant ranges.

    ``ap_ranges`` maps AP name to ``(x, y, z, slant_m)`` in the reading frame.
    """
    circles: list[tuple[float, float, float]] = []
    for ap_name, (ax, ay, az, slant) in ap_ranges.items():
        ri_sq = horizontal_radius_sq_from_slant(
            slant, device_z_m=device_z_m, ap_z_m=az
        )
        if ri_sq < min_horizontal_m * min_horizontal_m:
            raise ValueError(
                f"horizontal reach to {ap_name!r} is below {min_horizontal_m} m "
                f"(slant {slant:.2f} m with the configured heights)"
            )
        circles.append((ax, ay, ri_sq))

    if len(circles) < 2:
        return None

    if len(circles) == 2:
        return _intersect_two_circles(circles[0], circles[1], config=config)

    return _least_squares_trilateration(circles)


def infer_xy_from_single_slant_with_prior(
    ap_x: float,
    ap_y: float,
    ap_z: float,
    slant_m: float,
    *,
    device_z_m: float,
    prior_x: float,
    prior_y: float,
    config: LocatorConfig,
    min_horizontal_m: float = 0.5,
) -> tuple[float, float]:
    """
    Place the antenna on the horizontal circle around an AP at the laser slant range.

    One slant range only fixes distance, not bearing. When a second AP is not
    visible, pick the point on that circle that lies in the direction of
    ``prior_x/y`` (usually the instantaneous WiFi geometry estimate).
    """
    ri = math.sqrt(
        horizontal_radius_sq_from_slant(
            slant_m, device_z_m=device_z_m, ap_z_m=ap_z
        )
    )
    if ri < min_horizontal_m:
        raise ValueError(
            f"horizontal reach {ri:.2f} m is below minimum {min_horizontal_m} m"
        )

    dx = prior_x - ap_x
    dy = prior_y - ap_y
    norm = math.hypot(dx, dy)
    if norm < 0.5:
        cx = (config.width_m or 40.0) * 0.5
        cy = (config.height_m or 40.0) * 0.5
        dx = cx - ap_x
        dy = cy - ap_y
        norm = math.hypot(dx, dy)
        if norm < 0.5:
            dx, dy = 1.0, 0.0
            norm = 1.0

    x = ap_x + ri * dx / norm
    y = ap_y + ri * dy / norm
    return _clamp_xy_to_floor(x, y, config=config)


def _clamp_xy_to_floor(
    x: float,
    y: float,
    *,
    config: LocatorConfig,
) -> tuple[float, float]:
    if config.width_m is not None:
        x = min(max(x, 0.0), config.width_m)
    if config.height_m is not None:
        y = min(max(y, 0.0), config.height_m)
    return x, y


def _intersect_two_circles(
    c0: tuple[float, float, float],
    c1: tuple[float, float, float],
    *,
    config: LocatorConfig,
) -> tuple[float, float]:
    x0, y0, r0_sq = c0
    x1, y1, r1_sq = c1
    dx = x1 - x0
    dy = y1 - y0
    d = math.hypot(dx, dy)
    if d < 1e-9:
        raise ValueError("distance circles share the same center AP position")

    # No real intersection if circles are too far apart / nested.
    if d > math.sqrt(r0_sq) + math.sqrt(r1_sq):
        raise ValueError(
            "AP distance circles do not intersect; check laser ranges and AP coordinates"
        )
    if d < abs(math.sqrt(r0_sq) - math.sqrt(r1_sq)):
        raise ValueError(
            "AP distance circles do not intersect; one range may be too large"
        )

    a = (r0_sq - r1_sq + d * d) / (2 * d)
    h_sq = max(0.0, r0_sq - a * a)
    h = math.sqrt(h_sq)
    xm = x0 + a * dx / d
    ym = y0 + a * dy / d
    rx = -dy * h / d
    ry = dx * h / d
    p_a = (xm + rx, ym + ry)
    p_b = (xm - rx, ym - ry)
    return _pick_circle_solution(p_a, p_b, config=config)


def _pick_circle_solution(
    a: tuple[float, float],
    b: tuple[float, float],
    *,
    config: LocatorConfig,
) -> tuple[float, float]:
    def in_floor(p: tuple[float, float]) -> bool:
        x, y = p
        if config.width_m is not None and not (0.0 <= x <= config.width_m):
            return False
        if config.height_m is not None and not (0.0 <= y <= config.height_m):
            return False
        return True

    a_ok = in_floor(a)
    b_ok = in_floor(b)
    if a_ok and not b_ok:
        return a
    if b_ok and not a_ok:
        return b
    # Prefer the point closer to the floor-plan center when ambiguous.
    cx = (config.width_m or 0.0) * 0.5
    cy = (config.height_m or 0.0) * 0.5
    if config.width_m is None and config.height_m is None:
        return a
    return a if math.hypot(a[0] - cx, a[1] - cy) <= math.hypot(b[0] - cx, b[1] - cy) else b


def _least_squares_trilateration(
    circles: list[tuple[float, float, float]],
) -> tuple[float, float]:
    x0, y0, r0_sq = circles[0]
    ata_00 = ata_01 = ata_11 = 0.0
    atb_0 = atb_1 = 0.0
    for xi, yi, ri_sq in circles[1:]:
        a = 2.0 * (xi - x0)
        b = 2.0 * (yi - y0)
        rhs = (
            r0_sq
            - ri_sq
            + xi * xi
            - x0 * x0
            + yi * yi
            - y0 * y0
        )
        ata_00 += a * a
        ata_01 += a * b
        ata_11 += b * b
        atb_0 += a * rhs
        atb_1 += b * rhs

    det = ata_00 * ata_11 - ata_01 * ata_01
    if abs(det) < 1e-12:
        raise ValueError(
            "AP distance circles are degenerate; add another AP range or "
            "check coordinates"
        )
    x = (atb_0 * ata_11 - atb_1 * ata_01) / det
    y = (ata_00 * atb_1 - ata_01 * atb_0) / det
    return x, y
