"""Fuse noisy WiFi position fixes with robot motion.

A WiFi RSSI fix is a noisy, biased absolute position. A mobile robot also has
motion sources (a movement sensor, a base, a SLAM service) that are smooth and
locally accurate but drift. This module provides an adaptive per-axis Kalman
filter that predicts from motion between fixes and corrects with each WiFi fix,
plus small pure helpers for turning Viam motion readings into a ``MotionDelta``.

Everything here is pure Python (no numpy, no Viam imports) so it stays unit
testable; the Viam client calls live in the sensor model.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass
class MotionDelta:
    """Robot motion since the previous fix, in the WiFi floor-plan frame.

    ``dx_m`` / ``dy_m`` are only applied as a prediction step when
    ``has_direction`` is true (a frame-aligned source such as SLAM). Sources
    that only expose speed or a moving/stopped flag leave them at zero and
    instead drive ``speed_mps`` / ``is_moving`` so the filter can adapt how much
    it smooths.
    """

    dx_m: float = 0.0
    dy_m: float = 0.0
    speed_mps: float = 0.0
    is_moving: bool = True
    has_direction: bool = False
    sources: tuple[str, ...] = field(default_factory=tuple)


def slam_pose_delta(
    prev_xy_mm: tuple[float, float],
    curr_xy_mm: tuple[float, float],
    *,
    yaw_offset_deg: float = 0.0,
    scale: float = 1.0,
) -> tuple[float, float]:
    """Floor-frame (dx, dy) in meters between two SLAM poses given in millimeters.

    SLAM reports pose in its own map frame, which may be rotated/scaled relative
    to the WiFi floor plan. ``yaw_offset_deg`` rotates the motion delta into the
    floor frame and ``scale`` corrects any unit/scale mismatch. Defaults assume
    the frames are already aligned.
    """
    dx_mm = curr_xy_mm[0] - prev_xy_mm[0]
    dy_mm = curr_xy_mm[1] - prev_xy_mm[1]
    dx = dx_mm / 1000.0 * scale
    dy = dy_mm / 1000.0 * scale
    if yaw_offset_deg:
        theta = math.radians(yaw_offset_deg)
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        return dx * cos_t - dy * sin_t, dx * sin_t + dy * cos_t
    return dx, dy


def measurement_var_from_fix(
    *,
    base_noise_m: float,
    anchor_count: int,
    residual_rmse_m: float | None = None,
    fp_blend_weight: float = 0.0,
) -> float:
    """Estimate WiFi fix variance (m^2) from data the locator already returns.

    A fix backed by more anchors and a stronger fingerprint match is trusted
    more (smaller variance → larger Kalman gain); a large geometric residual
    widens it. The result is a 1-sigma standard deviation squared.
    """
    sigma = max(base_noise_m, 1e-3)
    # More anchors tighten the fix; 3 anchors is the configured floor.
    sigma *= math.sqrt(3.0 / max(anchor_count, 1))
    # A confident fingerprint blend shrinks the effective noise (cap at -50%).
    sigma *= 1.0 - 0.5 * max(0.0, min(1.0, fp_blend_weight))
    # A large weighted residual signals a poor geometric fit; widen the fix.
    if residual_rmse_m is not None and residual_rmse_m > base_noise_m:
        sigma += residual_rmse_m - base_noise_m
    return sigma * sigma


class PositionFilter:
    """Adaptive per-axis Kalman filter fusing motion prediction with WiFi fixes.

    The state is 2D position with independent (diagonal) variance per axis. The
    process noise grows with measured speed, so the estimate barely moves while
    the robot is stationary (heavy smoothing) and tracks quickly while driving.
    Fixes whose innovation exceeds ``max_innovation_m`` are rejected as outliers;
    after ``max_consecutive_rejects`` rejections the filter re-seeds to the
    measurement so it can recover from divergence or a relocated robot.
    """

    def __init__(
        self,
        *,
        process_noise_m: float = 0.5,
        measurement_noise_m: float = 3.0,
        max_innovation_m: float = 8.0,
        speed_scale: float = 1.0,
        init_variance_m2: float = 25.0,
        max_consecutive_rejects: int = 5,
    ) -> None:
        self.process_noise_m = max(process_noise_m, 0.0)
        self.measurement_noise_m = max(measurement_noise_m, 1e-3)
        self.max_innovation_m = max_innovation_m
        self.speed_scale = max(speed_scale, 0.0)
        self.init_variance_m2 = max(init_variance_m2, 1e-3)
        self.max_consecutive_rejects = max(max_consecutive_rejects, 1)
        self._x: float | None = None
        self._y: float | None = None
        self._px = self.init_variance_m2
        self._py = self.init_variance_m2
        self._rejects = 0

    @property
    def initialized(self) -> bool:
        return self._x is not None and self._y is not None

    @property
    def position(self) -> tuple[float, float] | None:
        if not self.initialized:
            return None
        return (self._x, self._y)  # type: ignore[return-value]

    def reset(self) -> None:
        self._x = None
        self._y = None
        self._px = self.init_variance_m2
        self._py = self.init_variance_m2
        self._rejects = 0

    def predict(self, motion: MotionDelta, dt_s: float) -> None:
        """Shift the estimate by directional motion and inflate uncertainty."""
        if not self.initialized:
            return
        if motion.has_direction:
            self._x += motion.dx_m  # type: ignore[operator]
            self._y += motion.dy_m  # type: ignore[operator]
        dt = max(dt_s, 0.0)
        speed = max(motion.speed_mps, 0.0) if motion.is_moving else 0.0
        step = self.process_noise_m + speed * self.speed_scale * dt
        q = step * step
        self._px += q
        self._py += q

    def update(
        self,
        meas_x: float,
        meas_y: float,
        *,
        measurement_var_m2: float | None = None,
        max_innovation_m: float | None = None,
    ) -> bool:
        """Correct with a WiFi fix. Returns False when the fix is gated out."""
        if measurement_var_m2 is None:
            r = self.measurement_noise_m * self.measurement_noise_m
        else:
            r = max(measurement_var_m2, 1e-6)

        if not self.initialized:
            self._x = meas_x
            self._y = meas_y
            self._px = r
            self._py = r
            self._rejects = 0
            return True

        gate = self.max_innovation_m if max_innovation_m is None else max_innovation_m
        innovation = math.hypot(meas_x - self._x, meas_y - self._y)  # type: ignore[operator]
        if gate is not None and gate > 0 and innovation > gate:
            self._rejects += 1
            if self._rejects < self.max_consecutive_rejects:
                return False
            # Persistent disagreement: trust the sensor and re-seed.
            self._x = meas_x
            self._y = meas_y
            self._px = r
            self._py = r
            self._rejects = 0
            return True

        self._rejects = 0
        kx = self._px / (self._px + r)
        ky = self._py / (self._py + r)
        self._x += kx * (meas_x - self._x)  # type: ignore[operator]
        self._y += ky * (meas_y - self._y)  # type: ignore[operator]
        self._px = (1.0 - kx) * self._px
        self._py = (1.0 - ky) * self._py
        return True
