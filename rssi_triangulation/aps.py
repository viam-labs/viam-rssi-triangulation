"""BSSID normalization and optional UniFi MAC variant matching."""

from __future__ import annotations

import re

# Last-octet offsets UniFi sometimes uses for other radios / WLANs.
UNIFI_RADIO_OCTET_DELTAS: tuple[int, ...] = (0, 1, 2, 3, 4)


def normalize_mac(mac: str) -> str:
    """Normalize MAC to lowercase colon-separated form."""
    cleaned = mac.strip().lower()
    m = re.match(r"^([0-9a-f]{2}(?::[0-9a-f]{2}){5})", cleaned)
    if m:
        cleaned = m.group(1)
    else:
        cleaned = cleaned.replace("-", ":")
    parts = cleaned.split(":")
    if len(parts) == 6 and all(len(p) <= 2 for p in parts):
        return ":".join(p.zfill(2) for p in parts)
    raise ValueError(f"invalid MAC address: {mac!r}")


def unifi_bssid_variants(base_mac: str) -> list[str]:
    """Likely BSSIDs for one UniFi AP (2.4/5 GHz and virtual WLAN offsets)."""
    base = normalize_mac(base_mac)
    octets = [int(p, 16) for p in base.split(":")]
    variants: list[str] = []
    for delta in UNIFI_RADIO_OCTET_DELTAS:
        last = octets[5] + delta
        if 0 <= last <= 255:
            mac = ":".join(f"{b:02x}" for b in octets[:5] + [last])
            if mac not in variants:
                variants.append(mac)
    return variants


def resolve_ap_name(bssid: str, lookup: dict[str, str]) -> str | None:
    return lookup.get(normalize_mac(bssid))
