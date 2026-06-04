"""AP registry types used during triangulation."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class AccessPoint:
    ap_name: str
    x_m: float
    y_m: float
    bssid: str


@dataclass(frozen=True)
class ApRegistry:
    scan_ssid: str
    access_points: tuple[AccessPoint, ...]
    unit: str = "m"

    @property
    def bssid_to_name(self) -> dict[str, str]:
        out: dict[str, str] = {}
        for ap in self.access_points:
            if ap.bssid in out and out[ap.bssid] != ap.ap_name:
                raise ValueError(
                    f"duplicate BSSID {ap.bssid!r}: {out[ap.bssid]!r} and {ap.ap_name!r}"
                )
            out[ap.bssid] = ap.ap_name
        return out

    @property
    def name_to_position(self) -> dict[str, tuple[float, float]]:
        return {ap.ap_name: (ap.x_m, ap.y_m) for ap in self.access_points}
