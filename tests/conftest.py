"""Shared fixtures for unit tests."""

from __future__ import annotations

import pytest

from rssi_triangulation.module_config import LocatorConfig, parse_config_dict, registry_from_config
from rssi_triangulation.registry import AccessPoint, ApRegistry


@pytest.fixture
def sample_config_dict() -> dict:
    return {
        "scan_ssid": "Viam-5G",
        "scan_count": 3,
        "floor_plan": {"x_origin_m": 1.0, "y_origin_m": 2.0},
        "access_points": [
            {"name": "AP-A", "x_m": 0.0, "y_m": 0.0, "bssid": "aa:bb:cc:dd:ee:01"},
            {"name": "AP-B", "x_m": 10.0, "y_m": 0.0, "bssid": "aa:bb:cc:dd:ee:02"},
            {"name": "AP-C", "x_m": 0.0, "y_m": 10.0, "bssid": "aa:bb:cc:dd:ee:03"},
        ],
    }


@pytest.fixture
def locator_config(sample_config_dict: dict) -> LocatorConfig:
    return parse_config_dict(sample_config_dict)


@pytest.fixture
def sample_registry(locator_config: LocatorConfig) -> ApRegistry:
    return registry_from_config(locator_config)


@pytest.fixture
def triangle_registry() -> ApRegistry:
    """Three APs forming a right triangle for centroid checks."""
    return ApRegistry(
        scan_ssid="TestNet",
        access_points=(
            AccessPoint(ap_name="left", x_m=0.0, y_m=0.0, bssid="aa:bb:cc:00:00:01"),
            AccessPoint(ap_name="right", x_m=10.0, y_m=0.0, bssid="aa:bb:cc:00:00:02"),
            AccessPoint(ap_name="top", x_m=0.0, y_m=10.0, bssid="aa:bb:cc:00:00:03"),
        ),
    )
