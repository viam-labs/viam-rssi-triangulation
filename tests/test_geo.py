from __future__ import annotations

import math

import pytest

from rssi_triangulation.geo import (
    horizontal_radius_sq_from_slant,
    infer_xy_from_single_slant_with_prior,
    infer_xy_from_slant_distances,
    slant_range_m,
)
from rssi_triangulation.module_config import parse_config_dict


def test_slant_range_matches_pythagoras() -> None:
    assert slant_range_m(
        0.0,
        0.0,
        device_z_m=0.2,
        ap_x=3.0,
        ap_y=4.0,
        ap_z=2.44,
    ) == pytest.approx(math.sqrt(3 * 3 + 4 * 4 + (2.44 - 0.2) ** 2), abs=0.01)


def test_infer_xy_from_two_ap_ranges() -> None:
    config = parse_config_dict(
        {
            "scan_ssid": "X",
            "scan_count": 1,
            "floor_plan": {"device_z_m": 0.2, "access_point_z_m": 2.44, "width_m": 40, "height_m": 40},
            "access_points": [
                {"name": "A", "x_m": 0.0, "y_m": 0.0, "bssid": "aa:bb:cc:dd:ee:01"},
                {"name": "B", "x_m": 10.0, "y_m": 0.0, "bssid": "aa:bb:cc:dd:ee:02"},
            ],
        }
    )
    true_x, true_y = 3.0, 4.0
    device_z = 0.2
    ap_z = 2.44
    slant_a = slant_range_m(true_x, true_y, device_z_m=device_z, ap_x=0.0, ap_y=0.0, ap_z=ap_z)
    slant_b = slant_range_m(true_x, true_y, device_z_m=device_z, ap_x=10.0, ap_y=0.0, ap_z=ap_z)
    xy = infer_xy_from_slant_distances(
        {
            "A": (0.0, 0.0, ap_z, slant_a),
            "B": (10.0, 0.0, ap_z, slant_b),
        },
        device_z_m=device_z,
        config=config,
    )
    assert xy is not None
    assert xy[0] == pytest.approx(true_x, abs=0.05)
    assert xy[1] == pytest.approx(true_y, abs=0.05)


def test_infer_xy_from_single_slant_with_prior() -> None:
    config = parse_config_dict(
        {
            "scan_ssid": "X",
            "scan_count": 1,
            "floor_plan": {"device_z_m": 0.2, "access_point_z_m": 2.44, "width_m": 40, "height_m": 40},
            "access_points": [
                {"name": "A", "x_m": 0.0, "y_m": 0.0, "bssid": "aa:bb:cc:dd:ee:01"},
            ],
        }
    )
    true_x, true_y = 0.0, 8.0
    device_z = 0.2
    ap_z = 2.44
    slant = slant_range_m(true_x, true_y, device_z_m=device_z, ap_x=0.0, ap_y=0.0, ap_z=ap_z)
    xy = infer_xy_from_single_slant_with_prior(
        0.0,
        0.0,
        ap_z,
        slant,
        device_z_m=device_z,
        prior_x=0.0,
        prior_y=1.0,
        config=config,
    )
    assert xy[0] == pytest.approx(0.0, abs=0.05)
    assert xy[1] == pytest.approx(true_y, abs=0.15)


def test_horizontal_radius_rejects_impossible_slant() -> None:
    with pytest.raises(ValueError, match="shorter than"):
        horizontal_radius_sq_from_slant(1.0, device_z_m=0.2, ap_z_m=2.44)
