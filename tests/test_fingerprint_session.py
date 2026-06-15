from __future__ import annotations

import pytest

from rssi_triangulation.fingerprint_session import (
    aggregate_rssi_samples,
    get_fingerprint_session_manager,
)
from rssi_triangulation.fingerprint_commands import execute_fingerprint_command
from rssi_triangulation.module_config import parse_config_dict


def test_aggregate_rssi_samples_mean_and_std() -> None:
    means, stats = aggregate_rssi_samples(
        [
            {"AP-A": -60.0, "AP-B": -70.0},
            {"AP-A": -62.0, "AP-B": -68.0},
            {"AP-A": -58.0, "AP-B": -72.0},
        ]
    )
    assert means["AP-A"] == pytest.approx(-60.0)
    assert means["AP-B"] == pytest.approx(-70.0)
    assert stats["AP-A"]["n"] == 3.0
    assert stats["AP-A"]["std_dbm"] > 0.0


def test_fingerprint_recording_session_start_stop(
    monkeypatch, tmp_path, sample_config_dict: dict
) -> None:
    manager = get_fingerprint_session_manager()
    manager.cancel()

    def fake_scan(*_args, **_kwargs):
        return (
            [("AP-A", -55.0, None), ("AP-B", -65.0, None)],
            "fake",
            [],
            1,
        )

    monkeypatch.setattr(
        "rssi_triangulation.fingerprint_commands.collect_matched_scan",
        lambda *a, **k: fake_scan(),
    )
    config = parse_config_dict(sample_config_dict)
    db_path = tmp_path / "fp.sqlite"

    from rssi_triangulation.fingerprint import FingerprintStore

    db = FingerprintStore(db_path)

    start = execute_fingerprint_command(
        {
            "command": "start_fingerprint_recording",
            "label": "desk",
            "min_samples": 3,
            "auto_sample": False,
        },
        config=config,
        db=db,
    )
    assert start["active"] is True

    for _ in range(3):
        execute_fingerprint_command(
            {"command": "sample_fingerprint_recording"},
            config=config,
            db=db,
        )

    stop = execute_fingerprint_command(
        {"command": "stop_fingerprint_recording"},
        config=config,
        db=db,
    )
    assert stop["ok"] is True
    assert stop["label"] == "desk"
    assert stop["sample_count"] == 3
    assert "AP-A" in stop["rssi_stats_by_ap"]
    assert stop["rssi_stats_by_ap"]["AP-A"]["n"] == 3.0

    record = db.list_all()[0]
    assert record.rssi_stats_by_ap is not None
    assert record.scan_count == 3
