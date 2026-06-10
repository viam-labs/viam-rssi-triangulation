from __future__ import annotations

import math

import pytest

from rssi_triangulation.calibrate import (
    PathLossSample,
    calibrate_from_fingerprints,
    fit_path_loss,
    fit_warnings,
    path_loss_rmse_db,
    per_ap_residuals,
    samples_from_fingerprints,
    should_auto_apply_path_loss_calibration,
    try_periodic_path_loss_calibration,
)
from rssi_triangulation.fingerprint import FingerprintStore
from rssi_triangulation.fingerprint_commands import execute_fingerprint_command
from rssi_triangulation.module_config import parse_config_dict


def _model_rssi(distance_m: float, *, tx: float, n: float) -> float:
    return tx - 10.0 * n * math.log10(distance_m)


def _synthetic_samples(
    *, tx: float, n: float, distances: list[float]
) -> list[PathLossSample]:
    return [
        PathLossSample(
            fingerprint_label=f"fp{i}",
            ap_name=f"AP-{i}",
            distance_m=d,
            rssi_dbm=_model_rssi(d, tx=tx, n=n),
        )
        for i, d in enumerate(distances)
    ]


def test_fit_recovers_known_parameters() -> None:
    samples = _synthetic_samples(tx=-38.0, n=3.2, distances=[1.0, 2.0, 5.0, 10.0, 20.0])
    fit = fit_path_loss(samples)
    assert fit.tx_power_dbm == pytest.approx(-38.0, abs=0.01)
    assert fit.path_loss_n == pytest.approx(3.2, abs=0.01)
    assert fit.rmse_db == pytest.approx(0.0, abs=0.01)
    assert fit.sample_count == 5


def test_fit_requires_enough_samples() -> None:
    samples = _synthetic_samples(tx=-40.0, n=2.5, distances=[1.0, 10.0])
    with pytest.raises(ValueError, match="at least"):
        fit_path_loss(samples)


def test_fit_requires_distance_spread() -> None:
    samples = _synthetic_samples(tx=-40.0, n=2.5, distances=[10.0, 10.1, 10.2, 10.3])
    with pytest.raises(ValueError, match="too similar"):
        fit_path_loss(samples)


def test_per_ap_residuals_flags_outlier_ap() -> None:
    samples = _synthetic_samples(tx=-40.0, n=2.5, distances=[1.0, 3.0, 8.0, 15.0])
    # One AP reads 15 dB hotter than the model predicts (e.g. wrong coords).
    bad = PathLossSample(
        fingerprint_label="fp-bad",
        ap_name="AP-bad",
        distance_m=10.0,
        rssi_dbm=_model_rssi(10.0, tx=-40.0, n=2.5) + 15.0,
    )
    rows = per_ap_residuals(samples + [bad], tx_power_dbm=-40.0, path_loss_n=2.5)
    assert rows[0]["ap_name"] == "AP-bad"
    assert rows[0]["mean_residual_db"] == pytest.approx(15.0, abs=0.01)


def test_fit_warnings_on_implausible_values() -> None:
    samples = _synthetic_samples(tx=-40.0, n=0.5, distances=[1.0, 3.0, 8.0, 15.0])
    fit = fit_path_loss(samples)
    warnings = fit_warnings(fit)
    assert any("path_loss_n" in w for w in warnings)


def test_samples_from_fingerprints_prefers_measured_distance(locator_config) -> None:
    from rssi_triangulation.fingerprint import FingerprintRecord

    record = FingerprintRecord(
        id=1,
        label="desk",
        x_m=2.0,
        y_m=2.0,
        z_m=0.0,
        rssi_by_ap={"AP-A": -60.0},
        recorded_at="now",
        scan_count=1,
        positioned=False,
        distances_by_ap={"AP-A": 8.3},
    )
    samples = samples_from_fingerprints(locator_config, [record])
    assert len(samples) == 1
    assert samples[0].distance_m == pytest.approx(8.3)


def test_samples_from_fingerprints_uses_reading_frame(locator_config) -> None:
    db_records = []
    from rssi_triangulation.fingerprint import FingerprintRecord

    # AP-A is at floor (0,0) → reading frame (-1,-2) with origin (1,2).
    db_records.append(
        FingerprintRecord(
            id=1,
            label="spot",
            x_m=2.0,
            y_m=2.0,
            z_m=0.0,
            rssi_by_ap={"AP-A": -60.0, "Unknown-AP": -50.0},
            recorded_at="now",
            scan_count=1,
        )
    )
    samples = samples_from_fingerprints(locator_config, db_records)
    assert len(samples) == 1
    assert samples[0].ap_name == "AP-A"
    assert samples[0].distance_m == pytest.approx(5.0)


def test_samples_skip_near_field(locator_config) -> None:
    from rssi_triangulation.fingerprint import FingerprintRecord

    record = FingerprintRecord(
        id=1,
        label="under-ap",
        x_m=-1.0,
        y_m=-2.0,
        z_m=0.0,
        rssi_by_ap={"AP-A": -35.0},
        recorded_at="now",
        scan_count=1,
    )
    assert samples_from_fingerprints(locator_config, [record]) == []


def test_calibrate_command_end_to_end(tmp_path) -> None:
    tx, n = -42.0, 3.0
    config = parse_config_dict(
        {
            "scan_ssid": "Net",
            "scan_count": 1,
            "access_points": [
                {"name": "A", "x_m": 0.0, "y_m": 0.0, "bssid": "aa:bb:cc:dd:ee:01"},
                {"name": "B", "x_m": 20.0, "y_m": 0.0, "bssid": "aa:bb:cc:dd:ee:02"},
                {"name": "C", "x_m": 0.0, "y_m": 20.0, "bssid": "aa:bb:cc:dd:ee:03"},
            ],
        }
    )
    ap_xy = {"A": (0.0, 0.0), "B": (20.0, 0.0), "C": (0.0, 20.0)}
    db = FingerprintStore(tmp_path / "fp.sqlite")
    for label, (fx, fy) in {"p1": (2.0, 2.0), "p2": (10.0, 5.0), "p3": (18.0, 16.0)}.items():
        rssi = {
            name: _model_rssi(math.hypot(fx - x, fy - y), tx=tx, n=n)
            for name, (x, y) in ap_xy.items()
        }
        db.record(label, x_m=fx, y_m=fy, rssi_by_ap=rssi, scan_count=1)

    result = execute_fingerprint_command(
        {"command": "calibrate_path_loss"},
        config=config,
        db=db,
        current_tx_power_dbm=-40.0,
        current_path_loss_n=2.5,
    )
    assert result["ok"] is True
    assert result["tx_power_dbm"] == pytest.approx(tx, abs=0.05)
    assert result["path_loss_n"] == pytest.approx(n, abs=0.01)
    assert result["sample_count"] == 9
    assert result["fingerprint_count"] == 3
    # The fitted model should beat the (wrong) current parameters.
    assert result["rmse_db"] < result["current"]["rmse_db"]


def test_calibrate_from_fingerprints_empty_db(tmp_path, locator_config) -> None:
    db = FingerprintStore(tmp_path / "fp.sqlite")
    with pytest.raises(ValueError, match="Record more fingerprints"):
        calibrate_from_fingerprints(locator_config, db)


def test_path_loss_rmse_db_zero_for_exact_model() -> None:
    samples = _synthetic_samples(tx=-40.0, n=2.5, distances=[1.0, 5.0, 12.0, 30.0])
    assert path_loss_rmse_db(samples, tx_power_dbm=-40.0, path_loss_n=2.5) == pytest.approx(
        0.0, abs=1e-9
    )


def test_should_auto_apply_rejects_implausible_fit() -> None:
    assert should_auto_apply_path_loss_calibration({"ok": True}) is True
    assert should_auto_apply_path_loss_calibration({"ok": False}) is False
    assert should_auto_apply_path_loss_calibration(
        {
            "ok": True,
            "warnings": [
                "fitted path_loss_n=0.50 is outside the plausible indoor range [1.5, 6.0]"
            ],
        }
    ) is False


def test_try_periodic_path_loss_calibration_runs_once_then_waits(tmp_path) -> None:
    tx, n = -42.0, 3.0
    config = parse_config_dict(
        {
            "scan_ssid": "Net",
            "scan_count": 1,
            "access_points": [
                {"name": "A", "x_m": 0.0, "y_m": 0.0, "bssid": "aa:bb:cc:dd:ee:01"},
                {"name": "B", "x_m": 20.0, "y_m": 0.0, "bssid": "aa:bb:cc:dd:ee:02"},
                {"name": "C", "x_m": 0.0, "y_m": 20.0, "bssid": "aa:bb:cc:dd:ee:03"},
            ],
        }
    )
    ap_xy = {"A": (0.0, 0.0), "B": (20.0, 0.0), "C": (0.0, 20.0)}
    db = FingerprintStore(tmp_path / "fp.sqlite")
    for label, (fx, fy) in {"p1": (2.0, 2.0), "p2": (10.0, 5.0), "p3": (18.0, 16.0)}.items():
        rssi = {
            name: _model_rssi(math.hypot(fx - x, fy - y), tx=tx, n=n)
            for name, (x, y) in ap_xy.items()
        }
        db.record(label, x_m=fx, y_m=fy, rssi_by_ap=rssi, scan_count=1)

    out_tx, out_n, last, result = try_periodic_path_loss_calibration(
        config,
        db,
        current_tx_power_dbm=-40.0,
        current_path_loss_n=2.5,
        last_calibration_monotonic=None,
        interval_s=3600.0,
        now=100.0,
    )
    assert result is not None
    assert result["applied"] is True
    assert out_tx == pytest.approx(tx, abs=0.05)
    assert out_n == pytest.approx(n, abs=0.01)
    assert last == 100.0

    _, _, last2, result2 = try_periodic_path_loss_calibration(
        config,
        db,
        current_tx_power_dbm=out_tx,
        current_path_loss_n=out_n,
        last_calibration_monotonic=last,
        interval_s=3600.0,
        now=200.0,
    )
    assert result2 is None
    assert last2 == last
