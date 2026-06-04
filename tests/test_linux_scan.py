from __future__ import annotations

from rssi_triangulation.linux_scan import parse_iw_scan, parse_wpa_cli_scan_results


IW_SAMPLE = """
BSS aa:bb:cc:dd:ee:01(on wlan0)
\tfreq: 5180
\tsignal: -65.00 dBm
\tSSID: Viam-5G
BSS aa:bb:cc:dd:ee:02(on wlan0)
\tfreq: 5180
\tsignal: -80.00 dBm
\tSSID: Viam-5G
"""

IW_STRONGEST_SIGNAL_WINS = """
BSS aa:bb:cc:dd:ee:01(on wlan0)
\tfreq: 5180
\tsignal: -70.00 dBm
\tsignal avg: -72.00 dBm
\tSSID: Viam-5G
"""

WPA_CLI_SAMPLE = """bssid / frequency / signal level / flags / ssid
aa:bb:cc:dd:ee:01\t5180\t-65\t[WPA2-PSK-CCMP][ESS]\tViam-5G
aa:bb:cc:dd:ee:02\t5180\t-80\t[WPA2-PSK-CCMP][ESS]\tViam-5G
"""


def test_parse_iw_scan_basic() -> None:
    readings = parse_iw_scan(IW_SAMPLE)
    assert len(readings) == 2
    by_bssid = {r.bssid: r for r in readings}
    assert by_bssid["aa:bb:cc:dd:ee:01"].rssi_dbm == -65.0
    assert by_bssid["aa:bb:cc:dd:ee:01"].ssid == "Viam-5G"
    assert by_bssid["aa:bb:cc:dd:ee:01"].frequency_mhz == 5180


def test_parse_iw_scan_keeps_strongest_signal_line() -> None:
    readings = parse_iw_scan(IW_STRONGEST_SIGNAL_WINS)
    assert len(readings) == 1
    assert readings[0].rssi_dbm == -70.0


def test_parse_wpa_cli_scan_results() -> None:
    readings = parse_wpa_cli_scan_results(WPA_CLI_SAMPLE)
    assert len(readings) == 2
    assert readings[0].bssid == "aa:bb:cc:dd:ee:01"
    assert readings[0].rssi_dbm == -65.0
    assert readings[0].ssid == "Viam-5G"
    assert readings[0].backend == "wpa_cli"
