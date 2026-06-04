# viam-rssi-triangulation

Uses **RSSI triangulation** to estimate floor-plan position from signal strength to known access points. Runs on Linux (e.g. an SBC) as a **Viam sensor module** or via a local test wrapper.

Works with any WiFi access point (UniFi, Cisco, Aruba, consumer mesh, etc.) as long as you can obtain and configure each AP’s **BSSID**, **position**, and the **SSID** you scan. 

## Requirements

- Linux with a WiFi interface that can scan
- AP locations (x, y coordinates from an origin point) on a floor plan in **meters**
- BSSIDs that match what the radio actually reports for your `scan_ssid` (see [Matching BSSIDs](#matching-bssids))

## Module config

The sensor and `test_scan_rssi.py` use the same JSON (office example: `examples/module_config_viam-5g.json`):

```json
{
  "scan_ssid": "MyNetwork",
  "scan_count": 5,
  "floor_plan": {
    "x_origin_m": 0,
    "y_origin_m": 0
  },
  "access_points": [
    {
      "name": "Lobby",
      "x_m": 12.5,
      "y_m": 8.0,
      "bssid": "aa:bb:cc:dd:ee:01"
    }
  ]
}
```

| Field | Meaning |
|-------|---------|
| `scan_ssid` | Only use scan results from this network name |
| `scan_count` | Number of scan passes to average per reading |
| `access_points[].name` | Label (must match across scans) |
| `access_points[].x_m`, `y_m` | AP position on your floor plan, in meters |
| `access_points[].bssid` | MAC address of that AP’s radio for this SSID |
| `floor_plan.x_origin_m`, `y_origin_m` | Subtracted from the estimated position (usually `0`) |

`get_readings()` returns `{ "position_x_m": …, "position_y_m": … }`.

Accuracy defaults:

- `method`: `hybrid` (weighted centroid + fingerprints when calibrated; falls back to centroid alone if DB empty)
- `fingerprint_max_blend`: `0.5`
- `min_anchors`: `3` (needs at least 3 known APs after filtering)
- `min_samples_per_ap`: auto (`2` when `scan_count >= 3`, else `1`)
- `max_rssi_delta_db`: `35` (drop anchors much weaker than the strongest)
- `min_rssi_dbm`: `-90` (drop very weak anchors)
- `weight_temperature`: `2.0` (softens RSSI weighting; see below)
- `smoothing_alpha`: `1.0` (no smoothing lag by default)
- `max_position_step_m`: `0` (no movement cap by default)

`weight_temperature` controls how sharply the weighted centroid favors the
strongest AP. `1.0` weights by raw received power, so the result snaps to
whichever AP is momentarily strongest and can jump several meters when that
ordering flips. Higher values flatten the weights so several nearby anchors
contribute, trading a little responsiveness for a much steadier position. The
default `2.0` is a good starting point; raise it if the position is still jumpy
while stationary, lower it toward `1.0` if it feels sluggish to follow you.

Optional Viam attributes: `interface`, `backend`, `scan_delay_s`, `blocking_scan`, `strict_mac`, `method`, `min_anchors`, `min_samples_per_ap`, `max_rssi_delta_db`, `min_rssi_dbm`, `weight_temperature`, `smoothing_alpha`, `max_position_step_m`, `tx_power_dbm`, `path_loss_n`, `fingerprint_db_path`, `fingerprint_k`, `fingerprint_min_common_aps`, `fingerprint_max_rms_db`, `fingerprint_fallback`.

### Matching BSSIDs

Default **`strict_mac: true`** (recommended): the scanned BSSID must **exactly** match an entry in `access_points`. Use your controller, `iw scan`, `wpa_cli scan_results`, or `sudo python3 test_scan_rssi.py --json --debug` to list every BSSID heard on the SSID.

Set **`strict_mac: false`** only if one physical AP advertises several related BSSIDs (common on some UniFi deployments). The module then also matches MACs with the same first five octets and last octet +0…+4. Other vendors usually keep `strict_mac: true`.

---

## Floor plan web tool (config helper)

Mark AP positions and BSSIDs, then export module config JSON.

### Run

```bash
cd web/floorplan
python3 -m http.server 8080
```

Open **http://localhost:8080** (or `http://<host-ip>:8080` on your LAN). Full steps are below.

### Steps

1. **Image** — Upload a to-scale floor plan (PNG/PDF with DPI metadata works best).
2. **Scale** — Set real-world scale (e.g. 1 inch = 20 feet). Use **output units: Meters**.
3. **Origin** — One click for (0, 0). +X right, +Y down.
4. **Access points** — Click each AP; set **name** and **BSSID**.
5. **Module config** — Set `scan_ssid`, `scan_count`, then copy or download JSON.

Save as e.g. `examples/my_site.json` or paste into Viam sensor attributes. **Import module config** reloads an existing file for edits.

### Finding BSSIDs

Use whatever your gear provides, for example:

- Controller UI (per-AP / per-radio BSSID)
- `sudo iw dev wlan0 scan` or `sudo wpa_cli -i wlan0 scan_results`
- Run `sudo python3 test_scan_rssi.py --json --debug` once and read the `heard` array (no AP match until config is filled in)

Each `access_points[].bssid` must be the address your client sees for that AP on `scan_ssid`.

---

## Local test on the SBC

```bash
cd ~/viam-rssi-triangulation
sudo python3 test_scan_rssi.py --config examples/module_config_viam-5g.json
```

```bash
sudo python3 test_scan_rssi.py --config path/to/config.json --json
# All BSSIDs on the SSID (for filling in config):
sudo python3 test_scan_rssi.py --config path/to/config.json --json --debug
sudo python3 test_scan_rssi.py --config path/to/config.json --scans 5 --scan-delay 0.2
```

The local wrapper uses the same conservative defaults as the module. To loosen filtering while diagnosing:

```bash
sudo python3 test_scan_rssi.py --config path/to/config.json --min-aps 2 --min-samples-per-ap 1 --max-rssi-delta 30 --min-rssi -90
```

Smoothing is opt-in. If position jumps while the robot is nearly stationary and you can tolerate lag, run continuously and set smoothing:

```bash
# More stable / slower to respond
sudo python3 test_scan_rssi.py --config path/to/config.json --interval 2 --smoothing-alpha 0.15 --max-position-step-m 0.5

# Explicit raw per-reading estimate (same as defaults)
sudo python3 test_scan_rssi.py --config path/to/config.json --interval 2 --smoothing-alpha 1 --max-position-step-m 0
```

If the interface is busy while connected:

```bash
sudo python3 test_scan_rssi.py --config path/to/config.json --backend wpa_cli
```

Scan backends (in order): `iw` → `wpa_cli` → `nmcli`. Repeat on an interval: `--interval 5`.

Non-UniFi with multiple BSSIDs per AP name: try `--no-strict-mac` on the test script, or `strict_mac: false` on the Viam component.

### Fingerprint calibration (optional, more accurate)

Stand under each configured AP and record a fingerprint (stored in SQLite next to your config by default):

```bash
sudo python3 test_scan_rssi.py --config examples/module_config_viam-5g.json \
  --record-fingerprint "Cafe, WoH1"
# repeat for each access_points[].name
sudo python3 test_scan_rssi.py --config examples/module_config_viam-5g.json --list-fingerprints
```

Then localize ( **`hybrid` is the default** — blends centroid with fingerprints when the DB has entries):

```bash
sudo python3 test_scan_rssi.py --config examples/module_config_viam-5g.json --interval 2
```

Pure fingerprint mode (snaps to calibrated points only):

```bash
sudo python3 test_scan_rssi.py --config examples/module_config_viam-5g.json \
  --method fingerprint --interval 2
```

Geometry only (no fingerprints): `--method weighted_centroid`. Tune blend with `--fingerprint-max-blend 0.5` (0 = centroid only, 1 = full fingerprint pull at max confidence).

**Speed:** `hybrid` does not add extra WiFi scans. Each reading runs `scan_count` full scans (often **~2s each** on a Pi with `iw`, or longer if `iw` retries “device busy” then falls back to `wpa_cli`). So `--scans 3` is often **~6–7s per cycle**; `--interval 0.2` only sleeps 0.2s *after* that work finishes.

**Fast scanning is the default** (`wpa_cli` first, short poll waits). For maximum RSSI stability when recording fingerprints, use `--thorough-scan` (slower, ~6–7s with `--scans 3`).

```bash
sudo python3 test_scan_rssi.py --scans 2 --interval 0.5
sudo python3 test_scan_rssi.py --record-fingerprint "Cafe, WoH1" --thorough-scan
```

Output includes `Cycle time: …s` so you can see actual duration. On the robot: `"fast_scan": true` (default); set `"thorough_scan": true` to opt into the slow path.

**Important:** AP-only fingerprints label each reading with the **nearest AP’s floor-plan
coordinates** (discrete points). With fast `--interval`, the winning AP can change scan-to-scan
and the position will jump. The output line `Fingerprint: <name>` shows which AP won; if you
see `weighted_centroid fallback`, the match was rejected (loosen `--fingerprint-max-rms` or
record more fingerprints). Matching uses **relative** RSSI (strongest AP = 0 dB) and requires
several overlapping APs by default (`--fingerprint-min-common-aps 3`).

On the robot, set `"fingerprint_db_path": "/path/to/fingerprints.sqlite"` (hybrid is already the default `method`). Calibrate via `do_command`:

```json
{ "command": "record_fingerprint", "ap_name": "Cafe, WoH1" }
```

Other commands: `list_fingerprints`, `delete_fingerprint` (`label`), `clear_fingerprints`, `record_fingerprint_here` (`label`, `x_m`, `y_m`).

---

## Automated tests

```bash
./scripts/run_tests.sh
```

On Raspberry Pi OS, if venv creation fails: `sudo apt install -y python3-venv python3-full`, then rerun.

---

## Viam module

Model: `viam-labs:rssi-triangulation:wifi-position` (sensor).

```bash
./setup.sh
viam module reload-local --part-id <part-id> \
  --model-name viam-labs:rssi-triangulation:wifi-position --name wifi-position
```

Paste your module config JSON into the component attributes in the Viam app.
