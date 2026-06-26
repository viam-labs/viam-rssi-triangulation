"""Fuse noisy WiFi/BLE position fixes with robot motion.

A WiFi or BLE RSSI fix is a noisy, biased absolute position. A mobile robot
also has motion sources (a movement sensor, a base, a SLAM service) that are
smooth and locally accurate but drift. This module provides a 4-state Extended
Kalman Filter (EKF) that predicts from IMU/motion between fixes and corrects
with each WiFi or BLE fix, plus small pure helpers for turning Viam motion
readings into a ``MotionDelta``.

State vector: ``[x, y, vx, vy]`` with a full 4×4 covariance matrix.

Everything here is pure Python (no numpy, no Viam imports) so it stays unit
testable; the Viam client calls live in the sensor model.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional

# Chi-squared gate threshold for 2 DOF at 95% confidence.  Innovations whose
# Mahalanobis distance squared (d² = νᵀ S⁻¹ ν) exceed this value are treated
# as outliers.  The Euclidean ``max_innovation_m`` is kept as a hard absolute
# cap (catches physically impossible jumps regardless of covariance).
_CHI2_2DOF_95: float = 5.991

# Type alias for 2-D matrices represented as lists of lists.
_Mat = List[List[float]]


# ---------------------------------------------------------------------------
# Pure-Python matrix helpers (4×4 / 2×2 — no numpy)
# ---------------------------------------------------------------------------


def _mat_mul(A: _Mat, B: _Mat) -> _Mat:
    """Generic matrix multiply for small dense matrices."""
    rows_a, cols_a = len(A), len(A[0])
    cols_b = len(B[0])
    return [
        [sum(A[i][k] * B[k][j] for k in range(cols_a)) for j in range(cols_b)]
        for i in range(rows_a)
    ]


def _mat_T(A: _Mat) -> _Mat:
    """Transpose."""
    return [[A[j][i] for j in range(len(A))] for i in range(len(A[0]))]


def _mat_add(A: _Mat, B: _Mat) -> _Mat:
    return [[A[i][j] + B[i][j] for j in range(len(A[0]))] for i in range(len(A))]


def _identity(n: int) -> _Mat:
    return [[1.0 if i == j else 0.0 for j in range(n)] for i in range(n)]


def _inv_2x2(M: _Mat) -> _Mat:
    """Inverse of a 2×2 matrix; clamps determinant away from zero."""
    det = M[0][0] * M[1][1] - M[0][1] * M[1][0]
    if abs(det) < 1e-12:
        det = math.copysign(1e-12, det) if det != 0 else 1e-12
    inv_det = 1.0 / det
    return [
        [M[1][1] * inv_det, -M[0][1] * inv_det],
        [-M[1][0] * inv_det, M[0][0] * inv_det],
    ]


# ---------------------------------------------------------------------------
# Public types
# ---------------------------------------------------------------------------


@dataclass
class SignalFix:
    """A position fix from any signal source (WiFi, BLE, …).

    This is the common currency passed between signal-source methods and the
    EKF update loop.  ``source`` identifies the signal type ("wifi" or "ble").
    ``measurement_var_m2`` is the per-fix noise variance fed to
    :meth:`PositionFilter.update`.  ``anchor_count`` is the number of anchors
    (APs or beacons) that contributed.  ``method`` is a short human-readable
    string describing the algorithm used (e.g. "centroid+fp",
    "ble-trilateration").  ``metadata`` is a pass-through dict used by the
    sensor model to build the full readings response (backend name, matched AP
    list, fingerprint match details, etc.).
    """

    x_m: float
    y_m: float
    measurement_var_m2: float
    source: str              # "wifi" | "ble"
    anchor_count: int
    method: str
    metadata: dict = field(default_factory=dict)


@dataclass
class MotionDelta:
    """Robot motion since the previous fix, in the WiFi floor-plan frame.

    ``dx_m`` / ``dy_m`` are positional displacement (from SLAM pose delta or
    IMU velocity × dt).  ``vx_m`` / ``vy_m`` carry the floor-frame IMU
    velocity when a MovementSensor provides orientation; the EKF predict step
    uses these to update the velocity states directly.  Sources that only
    expose speed or a moving/stopped flag leave the velocity fields at zero and
    set ``has_direction=False``.
    """

    dx_m: float = 0.0
    dy_m: float = 0.0
    vx_m: float = 0.0
    vy_m: float = 0.0
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
    residual_rmse_m: Optional[float] = None,
    fp_blend_weight: float = 0.0,
) -> float:
    """Estimate WiFi fix variance (m²) from data the locator already returns.

    A fix backed by more anchors and a stronger fingerprint match is trusted
    more (smaller variance → larger Kalman gain); a large geometric residual
    widens it. The result is a 1-sigma standard deviation squared.
    """
    sigma = max(base_noise_m, 1e-3)
    sigma *= math.sqrt(3.0 / max(anchor_count, 1))
    sigma *= 1.0 - 0.5 * max(0.0, min(1.0, fp_blend_weight))
    if residual_rmse_m is not None and residual_rmse_m > base_noise_m:
        sigma += residual_rmse_m - base_noise_m
    return sigma * sigma


# ---------------------------------------------------------------------------
# 4-state EKF
# ---------------------------------------------------------------------------


class PositionFilter:
    """4-state EKF (x, y, vx, vy) fusing motion prediction with position fixes.

    **Predict step** (every IMU/motion tick):
    The constant-velocity transition model propagates the state through
    ``F(dt)`` and inflates covariance by ``Q``.  When a ``MotionDelta`` with
    ``has_direction=True`` is supplied, the measured displacement overrides the
    filter's position prediction; when IMU velocity ``(vx_m, vy_m)`` is also
    set, the velocity states are pinned to those values.

    **Update step** (when a position fix arrives — WiFi or BLE):
    The measurement model ``H = [[1,0,0,0],[0,1,0,0]]`` selects position from
    the state.  Innovations are screened by a Mahalanobis chi-squared gate
    (2 DOF, default 95 % confidence) **and** a hard Euclidean cap
    (``max_innovation_m``).  After ``max_consecutive_rejects`` consecutive
    rejections the filter re-seeds to the measurement so it recovers from a
    sudden relocation.
    """

    def __init__(
        self,
        *,
        process_noise_m: float = 0.5,
        measurement_noise_m: float = 3.0,
        velocity_noise_mps: float = 0.1,
        max_innovation_m: float = 8.0,
        chi2_threshold: float = _CHI2_2DOF_95,
        speed_scale: float = 1.0,
        init_variance_m2: float = 25.0,
        init_velocity_variance_m2ps2: float = 0.25,
        max_consecutive_rejects: int = 5,
    ) -> None:
        self.process_noise_m = max(process_noise_m, 0.0)
        self.measurement_noise_m = max(measurement_noise_m, 1e-3)
        self.velocity_noise_mps = max(velocity_noise_mps, 0.0)
        self.max_innovation_m = max_innovation_m
        self.chi2_threshold = chi2_threshold
        self.speed_scale = max(speed_scale, 0.0)
        self.init_variance_m2 = max(init_variance_m2, 1e-3)
        self.init_velocity_variance_m2ps2 = max(init_velocity_variance_m2ps2, 0.0)
        self.max_consecutive_rejects = max(max_consecutive_rejects, 1)
        self._state: list[float] | None = None   # [x, y, vx, vy]
        self._P: _Mat = _identity(4)             # 4×4 covariance
        self._rejects: int = 0

    @property
    def initialized(self) -> bool:
        return self._state is not None

    @property
    def position(self) -> tuple[float, float] | None:
        if self._state is None:
            return None
        return (self._state[0], self._state[1])

    def reset(self) -> None:
        self._state = None
        self._P = _identity(4)
        self._rejects = 0

    def predict(self, motion: MotionDelta, dt_s: float) -> None:
        """Propagate the estimate forward using the constant-velocity model.

        The state is updated by ``F(dt) @ x`` then optionally overridden with
        the measured displacement/velocity from ``motion``.  Covariance grows
        according to ``Q`` (which scales with measured speed).
        """
        if not self.initialized:
            return

        dt = max(dt_s, 0.0)
        speed = max(motion.speed_mps, 0.0) if motion.is_moving else 0.0

        x = self._state  # type: ignore[assignment]

        # F(dt) @ x — constant-velocity propagation
        x_pred = [
            x[0] + x[2] * dt,
            x[1] + x[3] * dt,
            x[2],
            x[3],
        ]

        # Override position with measured displacement when available.
        # Override velocity states when IMU floor-frame velocity is present.
        if motion.has_direction:
            x_pred[0] = x[0] + motion.dx_m
            x_pred[1] = x[1] + motion.dy_m
            if motion.vx_m != 0.0 or motion.vy_m != 0.0:
                x_pred[2] = motion.vx_m
                x_pred[3] = motion.vy_m

        self._state = x_pred

        # Build F (constant-velocity transition)
        F: _Mat = [
            [1.0, 0.0,  dt, 0.0],
            [0.0, 1.0, 0.0,  dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]

        # Process noise Q (block-diagonal: position + velocity blocks)
        q_pos = (self.process_noise_m + speed * self.speed_scale * dt) ** 2
        q_vel = (self.velocity_noise_mps * dt) ** 2
        Q: _Mat = [
            [q_pos, 0.0,   0.0,   0.0  ],
            [0.0,   q_pos, 0.0,   0.0  ],
            [0.0,   0.0,   q_vel, 0.0  ],
            [0.0,   0.0,   0.0,   q_vel],
        ]

        # P = F @ P @ Fᵀ + Q
        FP = _mat_mul(F, self._P)
        self._P = _mat_add(_mat_mul(FP, _mat_T(F)), Q)

    def update(
        self,
        meas_x: float,
        meas_y: float,
        *,
        measurement_var_m2: float | None = None,
        max_innovation_m: float | None = None,
    ) -> bool:
        """Correct with a position fix (WiFi or BLE).

        Returns ``True`` when the fix was accepted (or used to re-seed after
        persistent rejection), ``False`` when it was gated out.

        The innovation is screened by both a Mahalanobis chi-squared gate and
        a hard Euclidean cap so that wildly wrong measurements never corrupt
        the estimate even if the covariance has collapsed.
        """
        r = (
            self.measurement_noise_m ** 2
            if measurement_var_m2 is None
            else max(measurement_var_m2, 1e-6)
        )

        if not self.initialized:
            self._state = [meas_x, meas_y, 0.0, 0.0]
            P0 = self.init_variance_m2
            Pv = self.init_velocity_variance_m2ps2
            self._P = [
                [r,   0.0, 0.0, 0.0],
                [0.0, r,   0.0, 0.0],
                [0.0, 0.0, Pv,  0.0],
                [0.0, 0.0, 0.0, Pv ],
            ]
            # Use init_variance_m2 for position if r < it (first seed may have
            # a tight measurement variance from a confident fingerprint fix).
            if P0 > r:
                self._P[0][0] = r
                self._P[1][1] = r
            self._rejects = 0
            return True

        P = self._P
        x = self._state  # type: ignore[assignment]

        # Innovation  ν = z − H·x  (H selects position states)
        innov = [meas_x - x[0], meas_y - x[1]]

        # Innovation covariance  S = H·P·Hᵀ + R  (top-left 2×2 of P plus R)
        S: _Mat = [
            [P[0][0] + r, P[0][1]    ],
            [P[1][0],     P[1][1] + r],
        ]
        S_inv = _inv_2x2(S)

        # Mahalanobis distance squared  d² = νᵀ S⁻¹ ν
        d2 = (
            innov[0] * (S_inv[0][0] * innov[0] + S_inv[0][1] * innov[1])
            + innov[1] * (S_inv[1][0] * innov[0] + S_inv[1][1] * innov[1])
        )

        # Chi-squared gate (primary) + hard Euclidean cap (secondary)
        chi2_gated = d2 > self.chi2_threshold
        gate = self.max_innovation_m if max_innovation_m is None else max_innovation_m
        euclid_gated = gate > 0 and math.hypot(innov[0], innov[1]) > gate

        if chi2_gated or euclid_gated:
            self._rejects += 1
            if self._rejects < self.max_consecutive_rejects:
                return False
            # Persistent disagreement → re-seed to the measurement
            self._state = [meas_x, meas_y, 0.0, 0.0]
            Pv = self.init_velocity_variance_m2ps2
            self._P = [
                [r,   0.0, 0.0, 0.0],
                [0.0, r,   0.0, 0.0],
                [0.0, 0.0, Pv,  0.0],
                [0.0, 0.0, 0.0, Pv ],
            ]
            self._rejects = 0
            return True

        self._rejects = 0

        # Kalman gain  K = P·Hᵀ·S⁻¹
        # P·Hᵀ = first two columns of P (H selects position states)
        PHt: _Mat = [[P[i][0], P[i][1]] for i in range(4)]
        K: _Mat = _mat_mul(PHt, S_inv)   # 4×2

        # State update  x = x + K·ν
        for i in range(4):
            self._state[i] = x[i] + K[i][0] * innov[0] + K[i][1] * innov[1]

        # Covariance update  P = (I − K·H)·P
        # (K·H)[i][j] = K[i][0] if j==0, K[i][1] if j==1, 0 otherwise
        # → (I − K·H)·P  row i  = P[i] − K[i][0]·P[0] − K[i][1]·P[1]
        P0_row = P[0][:]
        P1_row = P[1][:]
        self._P = [
            [P[i][j] - K[i][0] * P0_row[j] - K[i][1] * P1_row[j] for j in range(4)]
            for i in range(4)
        ]
        return True
