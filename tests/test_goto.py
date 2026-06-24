"""Navigation helpers for --goto."""

from __future__ import annotations

import pytest

from rssi_triangulation.fingerprint import FingerprintStore
from rssi_triangulation.locate import (
    PositionReading,
    compute_goto_guidance,
    goto_guidance_as_dict,
    resolve_goto_target,
)


def test_compute_goto_guidance_offset_and_arrival() -> None:
    guidance = compute_goto_guidance(
        label="desk",
        current=PositionReading(x_m=0.0, y_m=0.0),
        target_x_m=3.0,
        target_y_m=4.0,
        arrival_radius_m=1.0,
    )
    assert guidance.dx_m == pytest.approx(3.0)
    assert guidance.dy_m == pytest.approx(4.0)
    assert guidance.distance_m == pytest.approx(5.0)
    assert guidance.bearing_deg == pytest.approx(36.87, abs=0.1)
    assert guidance.arrived is False
    assert "+x" in guidance.hint and "+y" in guidance.hint

    arrived = compute_goto_guidance(
        label="desk",
        current=PositionReading(x_m=2.9, y_m=3.9),
        target_x_m=3.0,
        target_y_m=4.0,
        arrival_radius_m=1.0,
    )
    assert arrived.arrived is True
    assert "within" in arrived.hint


def test_goto_guidance_as_dict() -> None:
    guidance = compute_goto_guidance(
        label="desk",
        current=PositionReading(x_m=1.0, y_m=2.0),
        target_x_m=4.0,
        target_y_m=6.0,
    )
    row = goto_guidance_as_dict(guidance)
    assert row["label"] == "desk"
    assert row["target"]["x"] == 4.0
    assert row["delta_m"]["x"] == 3.0
    assert row["distance_m"] == pytest.approx(5.0)
    assert row["bearing_from"] == "+y"


def test_resolve_goto_target_requires_positioned(tmp_path) -> None:
    db = FingerprintStore(tmp_path / "fp.sqlite")
    db.record(
        "here",
        x_m=1.0,
        y_m=2.0,
        rssi_by_ap={"AP-A": -55.0},
        scan_count=1,
    )
    record = resolve_goto_target(db, "here")
    assert record.x_m == 1.0

    db.record(
        "rssi-only",
        rssi_by_ap={"AP-A": -55.0},
        scan_count=1,
        positioned=False,
    )
    with pytest.raises(ValueError, match="positioned=false"):
        resolve_goto_target(db, "rssi-only")

    with pytest.raises(ValueError, match="no fingerprint"):
        resolve_goto_target(db, "missing")
