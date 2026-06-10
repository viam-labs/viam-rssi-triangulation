"""Linux WiFi scan backends for RSSI collection."""

from __future__ import annotations

import re
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import Callable

from .aps import normalize_mac


@dataclass(frozen=True)
class WifiReading:
    bssid: str
    ssid: str | None
    rssi_dbm: float
    frequency_mhz: int | None
    backend: str


def _run(cmd: list[str], timeout: float = 30.0) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        cmd,
        capture_output=True,
        text=True,
        timeout=timeout,
        check=False,
    )


def _cmd_output(proc: subprocess.CompletedProcess[str]) -> str:
    return (proc.stderr or proc.stdout or "").strip()


def _is_device_busy(proc: subprocess.CompletedProcess[str]) -> bool:
    text = _cmd_output(proc).lower()
    return proc.returncode in (240, 16) or "busy" in text or "-16" in text


def detect_wireless_interface(preferred: str | None = None) -> str:
    if preferred:
        return preferred
    if shutil.which("iw"):
        proc = _run(["iw", "dev"], timeout=5.0)
        if proc.returncode == 0:
            for line in proc.stdout.splitlines():
                m = re.match(r"^\s+Interface\s+(\S+)", line)
                if m:
                    return m.group(1)
    return "wlan0"


def _filter_by_ssid(readings: list[WifiReading], network: str | None) -> list[WifiReading]:
    if not network:
        return readings
    want = network.strip()
    return [r for r in readings if (r.ssid or "").strip() == want]


def parse_iw_scan(output: str) -> list[WifiReading]:
    readings: list[WifiReading] = []
    current_bssid: str | None = None
    current_ssid: str | None = None
    current_rssi: float | None = None
    current_freq: int | None = None

    def flush() -> None:
        nonlocal current_bssid, current_ssid, current_rssi, current_freq
        if current_bssid is None or current_rssi is None:
            return
        readings.append(
            WifiReading(
                bssid=current_bssid,
                ssid=current_ssid,
                rssi_dbm=current_rssi,
                frequency_mhz=current_freq,
                backend="iw",
            )
        )

    for line in output.splitlines():
        bss = re.match(r"^BSS ([0-9a-f:]+(?:-[0-9a-f]+)?)", line, re.IGNORECASE)
        if bss:
            flush()
            current_bssid = normalize_mac(bss.group(1).split("(")[0])
            current_ssid = None
            current_rssi = None
            current_freq = None
            continue
        if current_bssid is None:
            continue
        if line.strip().startswith("freq:"):
            m = re.search(r"freq:\s*(\d+)", line)
            if m:
                current_freq = int(m.group(1))
        elif "signal:" in line:
            m = re.search(r"signal:\s*([-\d.]+)", line)
            if m:
                rssi = float(m.group(1))
                if current_rssi is None or rssi > current_rssi:
                    current_rssi = rssi
        elif line.strip().startswith("SSID:"):
            ssid = line.split("SSID:", 1)[1].strip()
            current_ssid = ssid or None

    flush()
    return readings


def scan_iw_blocking(interface: str) -> list[WifiReading]:
    """Blocking `iw scan` (slow; used by scan_mode=blocking)."""
    if not shutil.which("iw"):
        raise RuntimeError("iw not found on PATH")
    proc = _run(["iw", "dev", interface, "scan"], timeout=45.0)
    if proc.returncode != 0:
        raise RuntimeError(f"iw scan failed (exit {proc.returncode}): {_cmd_output(proc)}")
    return parse_iw_scan(proc.stdout)


def scan_iw_dump(interface: str) -> list[WifiReading]:
    proc = _run(["iw", "dev", interface, "scan", "dump"], timeout=8.0)
    if proc.returncode != 0:
        raise RuntimeError(f"iw scan dump failed: {_cmd_output(proc)}")
    return parse_iw_scan(proc.stdout)


def scan_iw_trigger(
    interface: str,
    *,
    max_wait_s: float = 3.0,
    poll_s: float = 0.12,
    min_results: int = 1,
    min_dwell_s: float = 0.4,
    stable_hits_required: int = 2,
    max_busy_attempts: int = 4,
) -> list[WifiReading]:
    """Trigger a scan and poll `scan dump` until results stabilize."""
    last_err = ""
    for attempt in range(max(1, max_busy_attempts)):
        trigger = _run(["iw", "dev", interface, "scan", "trigger"], timeout=5.0)
        if trigger.returncode == 0:
            break
        last_err = _cmd_output(trigger)
        if _is_device_busy(trigger):
            if attempt + 1 >= max_busy_attempts:
                raise RuntimeError(
                    "iw scan trigger failed: device busy (interface likely managed by "
                    f"NetworkManager while connected). Try --backend wpa_cli. "
                    f"Last error: {last_err}"
                )
            time.sleep(0.15 * (attempt + 1))
            continue
        raise RuntimeError(f"iw scan trigger failed: {last_err}")
    else:
        raise RuntimeError(
            "iw scan trigger failed: device busy (interface likely managed by "
            f"NetworkManager while connected). Try --backend wpa_cli or drop --backend iw. "
            f"Last error: {last_err}"
        )

    started = time.monotonic()
    deadline = started + max_wait_s
    last: list[WifiReading] = []
    stable_hits = 0
    while time.monotonic() < deadline:
        readings = scan_iw_dump(interface)
        if len(readings) >= min_results:
            if len(readings) == len(last):
                stable_hits += 1
                if (
                    stable_hits >= stable_hits_required
                    and (time.monotonic() - started) >= min_dwell_s
                ):
                    return readings
            else:
                stable_hits = 0
            last = readings
        time.sleep(poll_s)
    if last:
        return last
    return scan_iw_dump(interface)


def parse_wpa_cli_scan_results(output: str) -> list[WifiReading]:
    readings: list[WifiReading] = []
    for line in output.splitlines():
        line = line.strip()
        if not line or line.startswith("bssid") or line.startswith("Selected interface"):
            continue
        parts = line.split("\t")
        if len(parts) < 3:
            continue
        bssid = normalize_mac(parts[0].strip())
        try:
            frequency_mhz = int(parts[1].strip())
        except ValueError:
            frequency_mhz = None
        try:
            rssi_dbm = float(parts[2].strip())
        except ValueError:
            continue
        ssid = parts[4].strip() if len(parts) > 4 else None
        readings.append(
            WifiReading(
                bssid=bssid,
                ssid=ssid or None,
                rssi_dbm=rssi_dbm,
                frequency_mhz=frequency_mhz,
                backend="wpa_cli",
            )
        )
    return readings


def scan_wpa_cli(
    interface: str,
    *,
    max_wait_s: float = 2.5,
    poll_s: float = 0.12,
) -> list[WifiReading]:
    """Works when the interface is managed by wpa_supplicant / NetworkManager."""
    if not shutil.which("wpa_cli"):
        raise RuntimeError("wpa_cli not found on PATH")
    trigger = _run(["wpa_cli", "-i", interface, "scan"], timeout=10.0)
    if trigger.returncode != 0:
        raise RuntimeError(f"wpa_cli scan failed: {_cmd_output(trigger)}")

    deadline = time.monotonic() + max_wait_s
    while time.monotonic() < deadline:
        proc = _run(["wpa_cli", "-i", interface, "scan_results"], timeout=10.0)
        if proc.returncode != 0:
            raise RuntimeError(f"wpa_cli scan_results failed: {_cmd_output(proc)}")
        readings = parse_wpa_cli_scan_results(proc.stdout)
        if readings:
            return readings
        time.sleep(poll_s)

    proc = _run(["wpa_cli", "-i", interface, "scan_results"], timeout=10.0)
    if proc.returncode != 0:
        raise RuntimeError(f"wpa_cli scan_results failed: {_cmd_output(proc)}")
    return parse_wpa_cli_scan_results(proc.stdout)


def parse_nmcli_get_values(output: str) -> list[WifiReading]:
    readings: list[WifiReading] = []
    lines = [ln for ln in output.splitlines() if ln.strip()]
    i = 0
    while i + 2 < len(lines):
        bssid_raw, ssid, signal_raw = lines[i], lines[i + 1], lines[i + 2]
        i += 3
        try:
            quality = int(signal_raw)
            rssi_dbm = quality / 2 - 100
        except ValueError:
            continue
        try:
            bssid = normalize_mac(bssid_raw)
        except ValueError:
            continue
        readings.append(
            WifiReading(
                bssid=bssid,
                ssid=ssid or None,
                rssi_dbm=float(rssi_dbm),
                frequency_mhz=None,
                backend="nmcli",
            )
        )
    return readings


def scan_nmcli(interface: str) -> list[WifiReading]:
    if not shutil.which("nmcli"):
        raise RuntimeError("nmcli not found on PATH")
    proc = _run(
        ["nmcli", "-g", "BSSID,SSID,SIGNAL", "dev", "wifi", "list", "ifname", interface],
        timeout=30.0,
    )
    if proc.returncode != 0:
        raise RuntimeError(f"nmcli failed: {_cmd_output(proc)}")
    return parse_nmcli_get_values(proc.stdout)


def scan_nmcli_rescan_then_iw_dump(interface: str) -> list[WifiReading]:
    """Ask NetworkManager to rescan, then read results via iw dump."""
    if shutil.which("nmcli"):
        _run(["nmcli", "dev", "wifi", "rescan", "ifname", interface], timeout=15.0)
        time.sleep(0.5)
    return scan_iw_dump(interface)


ScanFn = Callable[[str], list[WifiReading]]

_CACHED_INTERFACE: str | None = None

def _scan_backend_pool(
    *,
    blocking: bool,
    fast: bool,
) -> list[tuple[str, ScanFn]]:
    if blocking:
        return [
            ("iw", scan_iw_blocking),
            ("wpa_cli", lambda iface: scan_wpa_cli(iface, max_wait_s=3.0)),
            ("nmcli", scan_nmcli),
        ]
    if fast:
        return [
            (
                "wpa_cli",
                lambda iface: scan_wpa_cli(iface, max_wait_s=1.2, poll_s=0.08),
            ),
            (
                "iw",
                lambda iface: scan_iw_trigger(
                    iface,
                    max_wait_s=1.0,
                    poll_s=0.08,
                    min_dwell_s=0.15,
                    stable_hits_required=1,
                    max_busy_attempts=1,
                ),
            ),
        ]
    return [
        ("iw", scan_iw_trigger),
        ("wpa_cli", scan_wpa_cli),
        ("nmcli+iw", scan_nmcli_rescan_then_iw_dump),
        ("nmcli", scan_nmcli),
    ]


def _backend_chain(pool: list[tuple[str, ScanFn]], backend: str | None) -> list[tuple[str, ScanFn]]:
    by_name = dict(pool)
    if not backend:
        return pool
    chain = [backend]
    if backend == "iw":
        chain.extend(["wpa_cli", "nmcli+iw", "nmcli"])
    elif backend == "wpa_cli":
        chain.extend(["iw", "nmcli+iw", "nmcli"])
    else:
        chain.extend([n for n in by_name if n != backend])
    seen: set[str] = set()
    out: list[tuple[str, ScanFn]] = []
    for name in chain:
        if name in seen or name not in by_name:
            continue
        seen.add(name)
        out.append((name, by_name[name]))
    return out


def scan_wifi(
    interface: str | None = None,
    network: str | None = None,
    backend: str | None = None,
    *,
    blocking: bool = False,
    fast: bool = False,
) -> tuple[list[WifiReading], str]:
    """
    Run a WiFi scan and return readings filtered to `network` SSID when set.

    Returns (readings, backend_name).
    """
    global _CACHED_INTERFACE
    if interface:
        iface = interface
        _CACHED_INTERFACE = interface
    elif _CACHED_INTERFACE:
        iface = _CACHED_INTERFACE
    else:
        iface = detect_wireless_interface(None)
        _CACHED_INTERFACE = iface

    errors: list[str] = []
    pool = _scan_backend_pool(blocking=blocking, fast=fast)
    backends = _backend_chain(pool, backend)

    for name, fn in backends:
        try:
            readings = _filter_by_ssid(fn(iface), network)
            if backend and name != backend:
                print(
                    f"note: --backend {backend} unavailable, used {name} instead",
                    file=sys.stderr,
                )
            return readings, name
        except Exception as exc:  # noqa: BLE001 — try next backend
            errors.append(f"{name}: {exc}")

    hint = (
        "\nTip: interface busy while connected is common — omit --backend iw so wpa_cli "
        "can be used, or run: --backend wpa_cli"
    )
    raise RuntimeError("no WiFi scan backend succeeded:\n  " + "\n  ".join(errors) + hint)
