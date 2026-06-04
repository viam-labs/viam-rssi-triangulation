from __future__ import annotations

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
    x, y = ap_position_in_reading_frame(config, "AP-B")
    assert x == pytest.approx(9.0)
    assert y == pytest.approx(-2.0)


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


def test_blend_pulls_toward_fingerprint() -> None:
    centroid = PositionReading(0.0, 0.0)
    fp = FingerprintMatch(
        x_m=10.0,
        y_m=0.0,
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


def test_fingerprint_confidence_zero_beyond_max_rms() -> None:
    fp = FingerprintMatch(
        x_m=0.0,
        y_m=0.0,
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
