from __future__ import annotations

import math

import pytest

from rssi_triangulation.fingerprint import (
    FingerprintStore,
    matched_to_rssi_dict,
    normalize_rssi_vector,
    rssi_vector_rms_db,
)
from rssi_triangulation.fingerprint_commands import (
    ap_position_in_reading_frame,
    execute_fingerprint_command,
)
from rssi_triangulation.locate import (
    PositionReading,
    blend_fingerprint_with_centroid,
    estimate_fingerprint_position,
    fingerprint_confidence,
    range_prior_blend_factor,
)
from rssi_triangulation.fingerprint import FingerprintMatch
from rssi_triangulation.module_config import parse_config_dict


def test_rssi_vector_rms_identical() -> None:
    a = {"AP-A": -60.0, "AP-B": -70.0}
    dist, common = rssi_vector_rms_db(a, a)
    assert dist == pytest.approx(0.0)
    assert common == 2


def test_fingerprint_store_record_and_match(tmp_path) -> None:
    db = FingerprintStore(tmp_path / "fp.sqlite")
    db.record(
        "kitchen",
        x_m=5.0,
        y_m=10.0,
        rssi_by_ap={"AP-A": -55.0, "AP-B": -65.0, "AP-C": -70.0},
        scan_count=3,
    )
    match = db.match(
        {"AP-A": -56.0, "AP-B": -64.0, "AP-C": -71.0},
        k=1,
        min_common_aps=2,
        max_rms_db=10.0,
    )
    assert match is not None
    assert match.x_m == pytest.approx(5.0)
    assert match.y_m == pytest.approx(10.0)
    assert match.label == "kitchen"


def test_fingerprint_replace_on_same_label(tmp_path) -> None:
    db = FingerprintStore(tmp_path / "fp.sqlite")
    db.record("a", x_m=1.0, y_m=2.0, rssi_by_ap={"AP-A": -50.0}, scan_count=1)
    db.record("a", x_m=3.0, y_m=4.0, rssi_by_ap={"AP-A": -60.0}, scan_count=2)
    assert db.count() == 1
    records = db.list_all()
    assert records[0].x_m == 3.0
    assert records[0].rssi_by_ap["AP-A"] == -60.0


def test_knn_interpolates_between_fingerprints(tmp_path) -> None:
    db = FingerprintStore(tmp_path / "fp.sqlite")
    db.record(
        "left",
        x_m=0.0,
        y_m=0.0,
        rssi_by_ap={"AP-A": -50.0, "AP-B": -80.0},
        scan_count=1,
    )
    db.record(
        "right",
        x_m=10.0,
        y_m=0.0,
        rssi_by_ap={"AP-A": -80.0, "AP-B": -50.0},
        scan_count=1,
    )
    match = db.match(
        {"AP-A": -65.0, "AP-B": -65.0},
        k=2,
        min_common_aps=2,
        max_rms_db=50.0,
    )
    assert match is not None
    assert match.x_m == pytest.approx(5.0, abs=1.5)
    assert match.k == 2


def test_ap_position_in_reading_frame(sample_config_dict: dict) -> None:
    config = parse_config_dict(sample_config_dict)
    x, y, z = ap_position_in_reading_frame(config, "AP-B")
    assert x == pytest.approx(9.0)
    assert y == pytest.approx(-2.0)
    assert z == pytest.approx(0.0)


def test_ap_position_includes_configured_z() -> None:
    config = parse_config_dict(
        {
            "scan_ssid": "X",
            "scan_count": 1,
            "access_point_z_m": 2.0,
            "access_points": [
                {
                    "name": "High",
                    "x_m": 5.0,
                    "y_m": 0.0,
                    "z_m": 5.5,
                    "bssid": "aa:bb:cc:dd:ee:00",
                },
            ],
        }
    )
    x, y, z = ap_position_in_reading_frame(config, "High")
    assert x == pytest.approx(5.0)
    assert z == pytest.approx(5.5)


def test_fingerprint_record_stores_z_m(tmp_path) -> None:
    db = FingerprintStore(tmp_path / "fp.sqlite")
    record = db.record(
        "spot",
        x_m=1.0,
        y_m=2.0,
        z_m=1.5,
        rssi_by_ap={"AP-A": -50.0},
        scan_count=1,
    )
    assert record.z_m == 1.5
    assert db.list_all()[0].z_m == 1.5


def test_list_and_clear_commands(tmp_path, sample_config_dict: dict) -> None:
    config = parse_config_dict(sample_config_dict)
    db = FingerprintStore(tmp_path / "fp.sqlite")
    db.record("AP-A", x_m=0.0, y_m=0.0, rssi_by_ap={"AP-A": -50.0}, scan_count=1)

    listed = execute_fingerprint_command(
        {"command": "list_fingerprints"},
        config=config,
        db=db,
    )
    assert listed["count"] == 1

    cleared = execute_fingerprint_command(
        {"command": "clear_fingerprints"},
        config=config,
        db=db,
    )
    assert cleared["removed"] == 1
    assert db.count() == 0


def test_normalized_matching_ignores_absolute_offset() -> None:
    a = {"AP-A": -50.0, "AP-B": -70.0, "AP-C": -80.0}
    b = {"AP-A": -60.0, "AP-B": -80.0, "AP-C": -90.0}
    dist, common = rssi_vector_rms_db(a, b, normalize=True, min_common_aps=3)
    assert dist == pytest.approx(0.0, abs=0.01)
    assert common == 3
    assert normalize_rssi_vector(a) == normalize_rssi_vector(b)


def test_min_common_fraction_blocks_weak_overlap(tmp_path) -> None:
    db = FingerprintStore(tmp_path / "fp.sqlite")
    db.record(
        "a",
        x_m=0.0,
        y_m=0.0,
        rssi_by_ap={"AP-A": -50.0, "AP-B": -60.0, "AP-C": -70.0},
        scan_count=1,
    )
    db.record(
        "b",
        x_m=10.0,
        y_m=0.0,
        rssi_by_ap={"AP-A": -80.0, "AP-B": -50.0, "AP-D": -70.0},
        scan_count=1,
    )
    # Only two APs overlap with fingerprint "a"; fraction rule requires >=3.
    assert (
        db.match(
            {"AP-A": -51.0, "AP-B": -59.0},
            min_common_aps=3,
            min_common_fraction=0.5,
            max_rms_db=50.0,
        )
        is None
    )


def test_fingerprint_match_averages_z_m(tmp_path) -> None:
    db = FingerprintStore(tmp_path / "fp.sqlite")
    db.record(
        "low",
        x_m=0.0,
        y_m=0.0,
        z_m=1.0,
        rssi_by_ap={"AP-A": -50.0, "AP-B": -80.0},
        scan_count=1,
    )
    db.record(
        "high",
        x_m=10.0,
        y_m=0.0,
        z_m=3.0,
        rssi_by_ap={"AP-A": -80.0, "AP-B": -50.0},
        scan_count=1,
    )
    match = db.match(
        {"AP-A": -65.0, "AP-B": -65.0},
        k=2,
        min_common_aps=2,
        max_rms_db=50.0,
    )
    assert match is not None
    assert match.z_m == pytest.approx(2.0, abs=0.5)


def test_blend_pulls_toward_fingerprint() -> None:
    centroid = PositionReading(0.0, 0.0, z_m=0.5)
    fp = FingerprintMatch(
        x_m=10.0,
        y_m=0.0,
        z_m=2.0,
        label="east",
        distance_db=2.0,
        common_aps=5,
        k=1,
        neighbors=("east",),
    )
    blended, annotated = blend_fingerprint_with_centroid(
        centroid,
        fp,
        max_blend=0.5,
        max_rms_db=10.0,
        min_common_aps=3,
    )
    assert annotated.blend_weight > 0
    assert 0 < blended.x_m < 10.0
    assert blended.y_m == pytest.approx(0.0)
    assert 0.5 < blended.z_m < 2.0


def test_fingerprint_confidence_zero_beyond_max_rms() -> None:
    fp = FingerprintMatch(
        x_m=0.0,
        y_m=0.0,
        z_m=0.0,
        label="a",
        distance_db=12.0,
        common_aps=4,
        k=1,
        neighbors=("a",),
    )
    assert fingerprint_confidence(fp, max_rms_db=10.0) == 0.0


def test_estimate_fingerprint_position_helper(tmp_path) -> None:
    db = FingerprintStore(tmp_path / "fp.sqlite")
    db.record(
        "here",
        x_m=12.0,
        y_m=34.0,
        rssi_by_ap={"a": -58.0, "b": -72.0},
        scan_count=1,
    )
    matched = [("a", -58.5, None), ("b", -71.0, None)]
    assert matched_to_rssi_dict(matched) == {"a": -58.5, "b": -71.0}
    est = estimate_fingerprint_position(
        db, matched, k=1, min_common_aps=2, min_common_fraction=0.0, max_rms_db=15.0
    )
    assert est is not None
    assert est.x_m == 12.0
    assert est.y_m == 34.0


def test_unpositioned_single_range_resolves_with_prior(
    tmp_path, sample_config_dict: dict
) -> None:
    config = parse_config_dict(sample_config_dict)
    db = FingerprintStore(tmp_path / "fp.sqlite")
    db.record(
        "desk",
        rssi_by_ap={"AP-A": -55.0, "AP-B": -65.0, "AP-C": -70.0},
        scan_count=1,
        positioned=False,
        distances_by_ap={"AP-A": 8.0},
    )
    matched = [("AP-A", -55.0, None), ("AP-B", -65.0, None), ("AP-C", -70.0, None)]
    est = estimate_fingerprint_position(
        db,
        matched,
        min_common_aps=2,
        max_rms_db=10.0,
        config=config,
        prior_xy=(0.0, 7.0),
    )
    assert est is not None
    assert est.positioned is True
    assert est.position_method == "range_prior"
    # AP-A is at (-1, -2) in the reading frame; 8 m slant with z=0 is radius 8.
    assert math.hypot(est.x_m + 1.0, est.y_m + 2.0) == pytest.approx(8.0, abs=0.1)


def test_range_prior_blend_scales_with_match_confidence() -> None:
    strong = FingerprintMatch(
        x_m=10.0,
        y_m=0.0,
        z_m=2.0,
        label="east",
        distance_db=3.76,
        common_aps=5,
        k=1,
        neighbors=("east",),
        position_method="range_prior",
    )
    weak = FingerprintMatch(
        x_m=10.0,
        y_m=0.0,
        z_m=2.0,
        label="east",
        distance_db=9.0,
        common_aps=3,
        k=1,
        neighbors=("east",),
        position_method="range_prior",
    )
    assert range_prior_blend_factor(strong, max_rms_db=10.0) > range_prior_blend_factor(
        weak, max_rms_db=10.0
    )
    assert range_prior_blend_factor(strong, max_rms_db=10.0) == pytest.approx(0.94, abs=0.02)

    centroid = PositionReading(0.0, 0.0, z_m=0.5)
    strong_blend, strong_ann = blend_fingerprint_with_centroid(
        centroid, strong, max_blend=0.5, max_rms_db=10.0, min_common_aps=3
    )
    weak_blend, weak_ann = blend_fingerprint_with_centroid(
        centroid, weak, max_blend=0.5, max_rms_db=10.0, min_common_aps=3
    )
    assert strong_ann.blend_weight > weak_ann.blend_weight
    assert strong_blend.x_m > weak_blend.x_m


def test_unpositioned_fingerprint_matches_by_label(tmp_path) -> None:
    db = FingerprintStore(tmp_path / "fp.sqlite")
    db.record(
        "desk",
        rssi_by_ap={"AP-A": -55.0, "AP-B": -65.0, "AP-C": -70.0},
        scan_count=1,
        positioned=False,
        distances_by_ap={"AP-A": 8.0},
    )
    match = db.match(
        {"AP-A": -55.0, "AP-B": -65.0, "AP-C": -70.0},
        min_common_aps=2,
        max_rms_db=10.0,
    )
    assert match is not None
    assert match.label == "desk"
    assert match.positioned is False
    assert match.x_m == 0.0
    assert match.y_m == 0.0


def test_record_fingerprint_uses_device_z_not_ap_z(
    monkeypatch, tmp_path, sample_config_dict
) -> None:
    def fake_scan(*_args, **_kwargs):
        return ([("AP-A", -60.0, None)], "mock", [], 1)

    monkeypatch.setattr(
        "rssi_triangulation.fingerprint_commands.collect_matched_scan",
        fake_scan,
    )
    config_dict = {
        **sample_config_dict,
        "floor_plan": {
            **sample_config_dict["floor_plan"],
            "device_z_m": 0.2,
            "access_point_z_m": 2.44,
        },
    }
    config = parse_config_dict(config_dict)
    db = FingerprintStore(tmp_path / "fp.sqlite")
    result = execute_fingerprint_command(
        {"command": "record_fingerprint", "ap_name": "AP-A"},
        config=config,
        db=db,
    )
    assert result["z_m"] == pytest.approx(0.2)
    assert db.list_all()[0].z_m == pytest.approx(0.2)


def test_record_fingerprint_rssi_infers_xy_with_two_ranges(
    monkeypatch, tmp_path
) -> None:
    def fake_scan(*_args, **_kwargs):
        return (
            [("A", -60.0, None), ("B", -70.0, None)],
            "mock",
            [],
            3,
        )

    monkeypatch.setattr(
        "rssi_triangulation.fingerprint_commands.collect_matched_scan",
        fake_scan,
    )
    config = parse_config_dict(
        {
            "scan_ssid": "X",
            "scan_count": 1,
            "floor_plan": {
                "device_z_m": 0.2,
                "access_point_z_m": 2.44,
                "width_m": 40,
                "height_m": 40,
            },
            "access_points": [
                {"name": "A", "x_m": 0.0, "y_m": 0.0, "bssid": "aa:bb:cc:dd:ee:01"},
                {"name": "B", "x_m": 10.0, "y_m": 0.0, "bssid": "aa:bb:cc:dd:ee:02"},
            ],
        }
    )
    from rssi_triangulation.geo import slant_range_m

    true_x, true_y = 3.0, 4.0
    slant_a = slant_range_m(true_x, true_y, device_z_m=0.2, ap_x=0.0, ap_y=0.0, ap_z=2.44)
    slant_b = slant_range_m(true_x, true_y, device_z_m=0.2, ap_x=10.0, ap_y=0.0, ap_z=2.44)
    db = FingerprintStore(tmp_path / "fp.sqlite")
    result = execute_fingerprint_command(
        {
            "command": "record_fingerprint_rssi",
            "label": "desk",
            "distance_to_ap": [
                {"ap_name": "A", "distance_m": slant_a},
                {"ap_name": "B", "distance_m": slant_b},
            ],
        },
        config=config,
        db=db,
    )
    assert result["positioned"] is True
    assert result["inferred_position"] is True
    assert result["x_m"] == pytest.approx(true_x, abs=0.05)
    assert result["y_m"] == pytest.approx(true_y, abs=0.05)
    assert result["z_m"] == pytest.approx(0.2)


def test_record_fingerprint_rssi_command(monkeypatch, tmp_path, sample_config_dict) -> None:
    def fake_scan(*_args, **_kwargs):
        return (
            [("AP-A", -60.0, None), ("AP-B", -70.0, None)],
            "mock",
            [],
            3,
        )

    monkeypatch.setattr(
        "rssi_triangulation.fingerprint_commands.collect_matched_scan",
        fake_scan,
    )
    config = parse_config_dict(sample_config_dict)
    db = FingerprintStore(tmp_path / "fp.sqlite")
    result = execute_fingerprint_command(
        {
            "command": "record_fingerprint_rssi",
            "label": "Matt Desk",
            "distance_to_ap": [
                {"ap_name": "AP-A", "distance_m": 8.3},
            ],
        },
        config=config,
        db=db,
    )
    assert result["ok"] is True
    assert result["positioned"] is False
    assert result["distances_m"] == {"AP-A": 8.3}
    stored = db.list_all()[0]
    assert stored.positioned is False
    assert stored.distances_by_ap == {"AP-A": 8.3}
