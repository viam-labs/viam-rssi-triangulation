from __future__ import annotations

import pytest

from rssi_triangulation.aps import normalize_mac, resolve_ap_name, unifi_bssid_variants


def test_normalize_mac_colon_form() -> None:
    assert normalize_mac("AA:BB:CC:DD:EE:FF") == "aa:bb:cc:dd:ee:ff"


def test_normalize_mac_dash_form() -> None:
    assert normalize_mac("aa-bb-cc-dd-ee-ff") == "aa:bb:cc:dd:ee:ff"


def test_normalize_mac_invalid() -> None:
    with pytest.raises(ValueError):
        normalize_mac("not-a-mac")


def test_unifi_bssid_variants() -> None:
    variants = unifi_bssid_variants("70:a7:41:65:12:21")
    assert "70:a7:41:65:12:21" in variants
    assert "70:a7:41:65:12:22" in variants
    assert len(variants) == 5


def test_resolve_ap_name() -> None:
    lookup = {"aa:bb:cc:dd:ee:01": "Kitchen"}
    assert resolve_ap_name("AA:BB:CC:DD:EE:01", lookup) == "Kitchen"
    assert resolve_ap_name("aa:bb:cc:dd:ee:99", lookup) is None
