from __future__ import annotations

import pytest

from rssi_triangulation.module_config import load_config_file, parse_config_dict


def test_parse_config_dict(sample_config_dict: dict) -> None:
    cfg = parse_config_dict(sample_config_dict)
    assert cfg.scan_ssid == "Viam-5G"
    assert cfg.scan_count == 3
    assert cfg.x_origin_m == 1.0
    assert cfg.y_origin_m == 2.0
    assert len(cfg.access_points) == 3
    assert cfg.access_points[0].bssid == "aa:bb:cc:dd:ee:01"


def test_parse_config_defaults_floor_origin() -> None:
    cfg = parse_config_dict(
        {
            "scan_ssid": "X",
            "scan_count": 1,
            "access_points": [
                {"name": "A", "x_m": 0, "y_m": 0, "bssid": "aa:bb:cc:dd:ee:00"},
            ],
        }
    )
    assert cfg.x_origin_m == 0.0
    assert cfg.y_origin_m == 0.0


def test_parse_config_normalizes_bssid() -> None:
    cfg = parse_config_dict(
        {
            "scan_ssid": "X",
            "scan_count": 1,
            "access_points": [
                {"name": "A", "x_m": 0, "y_m": 0, "bssid": "AA-BB-CC-DD-EE-FF"},
            ],
        }
    )
    assert cfg.access_points[0].bssid == "aa:bb:cc:dd:ee:ff"


@pytest.mark.parametrize(
    "payload",
    [
        {},
        {"scan_ssid": "X"},
        {"scan_ssid": "X", "scan_count": 1},
        {"scan_ssid": "X", "scan_count": 0, "access_points": []},
        {"scan_ssid": "X", "scan_count": 1, "access_points": []},
    ],
)
def test_parse_config_validation(payload: dict) -> None:
    with pytest.raises(ValueError):
        parse_config_dict(payload)


def test_load_config_file(examples_dir: None = None) -> None:
    from pathlib import Path

    path = Path(__file__).resolve().parents[1] / "examples" / "module_config_viam-5g.json"
    if not path.exists():
        pytest.skip("example config not present")
    cfg = load_config_file(path)
    assert cfg.scan_ssid == "Viam-5G"
    assert len(cfg.access_points) >= 2
