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
    "y_origin_m": 0,
    "device_z_m": 1.2,
    "access_point_z_m": 2.5
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
| `floor_plan.device_z_m` (or top-level `device_z_m`) | Antenna height above floor, in meters (`0`); used in 3D range math |
| `floor_plan.access_point_z_m` (or top-level) | Default AP mount height; per-AP override with `access_points[].z_m` |

## Methods

The sensor exposes two APIs: **`get_readings()`** for live position, and **`do_command()`** for fingerprint calibration. Both are also available via the local **`test_scan_rssi.py`** wrapper.

### `get_readings()`

Scans WiFi, estimates position, and returns coordinates in the configured floor-plan frame (after `floor_plan` origin subtraction).

**Response:**

```json
{
  "location": {
    "x": 12.34,
    "y": 56.78,
    "z": 1.2,
    "unit": "meters"
  },
  "access_points": [
    {
      "name": "WoStairsY",
      "x": 10.57,
      "y": -52.92,
      "z": 1.3,
      "unit": "meters",
      "bssid": "be:9c:6c:2e:de:2c",
      "rssi": -67.0
    }
  ],
  "method": "hybrid",
  "nearest_fingerprint": "Matt Desk",
  "fingerprint_match": {
    "label": "Matt Desk",
    "distance_db": 4.2,
    "common_aps": 6,
    "neighbors": ["Matt Desk"],
    "blend_weight": 0.35,
    "positioned": false
  },
  "fingerprint_rankings": [
    { "label": "Matt Desk", "distance_db": 4.2, "common_aps": 6, "positioned": false },
    { "label": "Lobby", "distance_db": 11.8, "common_aps": 5, "positioned": true }
  ]
}
```

When a fingerprint DB is configured, **`nearest_fingerprint`** is the best RSSI match (lowest `distance_db` RMS). **`fingerprint_rankings`** lists the top matches. RSSI-only fingerprints (`positioned: false`) participate in matching but do not pull `(x, y)` unless they have floor coordinates.

`location.z` is the device/antenna height above the floor (from `device_z_m` in config, or updated at runtime via **`set_device_z_m`**). Positioning uses **3D slant range** when AP and device heights differ: standing under a ceiling AP no longer looks meters away in x/y just because the radio is 2.5 m above you. `access_points` lists configured APs heard on this scan, **strongest RSSI first**. Each `x` / `y` / `z` is the offset from your estimated position to that AP (AP position minus current position), in meters — not absolute floor coordinates.

There is one positioning behavior: a **weighted centroid** (with 3D path-loss refinement), automatically **blended with fingerprint matches** in proportion to their confidence when a calibration DB exists. With no fingerprints it's pure geometry; the reported `method` field tells you what happened on each reading (`weighted_centroid`, `hybrid`, or `fingerprint`). See [Positioning options](#positioning-options) for tuning.

**Viam SDK (Python):**

```python
from viam.components.sensor import Sensor
from viam.services.machine import Machine

async with Machine.create_from_address("<machine-address>", "<api-key>", "<api-key-id>") as machine:
    sensor = Sensor.from_name("wifi-position")
    readings = await sensor.get_readings()
    loc = readings["location"]
    print(loc["x"], loc["y"], loc["z"], loc["unit"])
```

**Local wrapper:**

```bash
sudo python3 test_scan_rssi.py --config examples/module_config_viam-5g.json
sudo python3 test_scan_rssi.py --config examples/module_config_viam-5g.json --json
sudo python3 test_scan_rssi.py --config examples/module_config_viam-5g.json --interval 2
```

### `do_command()` (fingerprint calibration)

Use the sensor’s **`do_command`** API to record and manage RSSI fingerprints in a local SQLite database. Fingerprints improve `hybrid` and `fingerprint` positioning; with an empty DB, `hybrid` behaves like weighted centroid.

Set **`fingerprint_db_path`** on the component (default: `fingerprints.sqlite` in the module working directory). On a robot, use a persistent path, for example:

```json
"fingerprint_db_path": "/root/.viam/module-data/fingerprints.sqlite"
```

(`VIAM_MODULE_DATA` is set by `viam-server` for each module instance.)

#### Workflow

1. Configure `scan_ssid`, `access_points`, and other attributes as usual.
2. Stand under each AP (or at a known spot) and call **`record_fingerprint`** (or **`record_fingerprint_here`**).
3. Call **`list_fingerprints`** to verify entries.
4. **`get_readings`** uses the DB automatically whenever it has entries.
5. Once you have a handful of fingerprints, path-loss calibration runs **automatically** (default: every hour, first reading sooner) and applies fitted `tx_power_dbm` / `path_loss_n` in memory. You can also run **`calibrate_path_loss`** manually and persist the values in the component config if you want them fixed across restarts.

Re-record an AP by running **`record_fingerprint`** again with the same name (replaces the row). Use **`--scan-mode thorough`** on the local wrapper, or set **`scan_mode: "thorough"`** on the component, for steadier RSSI while calibrating.

#### Commands

Every request must include a **`command`** string. All responses include **`ok`** (boolean) and **`command`**.

| `command` | Purpose |
|-----------|---------|
| `record_fingerprint` | Scan WiFi and store a fingerprint at a configured AP’s floor-plan position |
| `record_fingerprint_here` | Scan and store at explicit coordinates (same frame as `get_readings`) |
| `record_fingerprint_rssi` | Scan and store RSSI only (no floor x/y); optional measured distance(s) to AP(s) for calibration |
| `list_fingerprints` | List all stored fingerprints |
| `delete_fingerprint` | Remove one fingerprint by label |
| `clear_fingerprints` | Remove all fingerprints |
| `calibrate_path_loss` | Fit `tx_power_dbm` / `path_loss_n` from stored fingerprints |
| `match_fingerprints` | Scan WiFi and return the nearest stored fingerprint(s) by RSSI similarity |
| `set_device_z_m` | Set device/antenna height in meters for subsequent `get_readings` |

**`set_device_z_m`** — update antenna height without restarting the module:

```json
{
  "command": "set_device_z_m",
  "z_m": 1.2
}
```

Response: `{ "ok": true, "command": "set_device_z_m", "z_m": 1.2 }`.

**`record_fingerprint`** — stand under the AP; position is taken from `access_points[]` (minus `floor_plan` origin, including `z_m`):

```json
{
  "command": "record_fingerprint",
  "ap_name": "Cafe, WoH1"
}
```

`ap_name` must match `access_points[].name` exactly. Optional: **`scan_count`** (overrides component `scan_count` for this scan only).

**`record_fingerprint_rssi`** — RSSI at a named spot without floor-plan coordinates. Optional laser/rangefinder distances improve path-loss calibration and, with a single range, enable approximate x/y blending at runtime via the geometry prior.

```json
{
  "command": "record_fingerprint_rssi",
  "label": "Matt Desk",
  "distances_m": { "SoA1": 8.3, "NoE4": 12.1 }
}
```

Or:

```json
{
  "command": "record_fingerprint_rssi",
  "label": "Matt Desk",
  "distance_to_ap": [
    { "ap_name": "SoA1", "distance_m": 8.3 }
  ]
}
```

Each `distance_to_ap` / `distances_m` entry must name a configured AP that was **heard in the same scan**. Distances are **3D slant ranges from the antenna** (`device_z_m` to `access_point_z_m`), not horizontal map distance.

A **single** range does not fix bearing (only a circle around the AP). At runtime the matcher projects that circle in the direction of the live WiFi geometry estimate, then blends toward that point (`position_method: range_prior`). Blend strength rises with RSSI match quality — a tight desk match (low RMS) pulls almost as hard as a surveyed coordinate; a loose match stays cautious. **Two or more** ranges can infer `(x, y)` via trilateration when circles intersect; otherwise the same single-range projection applies per AP. Stored coordinates from a laser survey are still preferred when you have them.

**`record_fingerprint_here`** — for grid points between APs:

```json
{
  "command": "record_fingerprint_here",
  "label": "hallway-mid",
  "x_m": 12.0,
  "y_m": 5.5
}
```

`x_m` / `y_m` are in the same coordinate frame as **`get_readings`** output (after origin subtraction). Optional **`z_m`** defaults to the current device height (`device_z_m` or the value set by **`set_device_z_m`**).

**`list_fingerprints`**:

```json
{ "command": "list_fingerprints" }
```

**`delete_fingerprint`**:

```json
{ "command": "delete_fingerprint", "label": "Cafe, WoH1" }
```

(`ap_name` is accepted as an alias for `label`.)

**`clear_fingerprints`**:

```json
{ "command": "clear_fingerprints" }
```

**`match_fingerprints`** — scan WiFi and rank stored fingerprints by RSSI similarity (lower `distance_db` = closer match). Optional **`k`** (default 5) controls how many rankings are returned. Works with RSSI-only desk fingerprints (no floor x/y).

```json
{ "command": "match_fingerprints", "k": 5 }
```

**`calibrate_path_loss`** — fit the log-distance model to the fingerprint DB. Every stored fingerprint is an RSSI vector at a known position, so each (fingerprint, AP) pair gives one RSSI-at-known-distance observation; the command least-squares fits `rssi = tx_power_dbm - 10 * path_loss_n * log10(distance)` over all of them. No scan is performed.

```json
{ "command": "calibrate_path_loss" }
```

Add `"apply": true` to also use the fitted values immediately for subsequent `get_readings` (until restart — persist them as `tx_power_dbm` / `path_loss_n` component attributes to keep them):

```json
{ "command": "calibrate_path_loss", "apply": true }
```

Requires at least 4 (fingerprint, AP) samples with a ≥2x spread in distances — in practice, a few fingerprints recorded both near and far from APs. The more fingerprints (and the better spread across the floor), the better the fit.

#### Example responses

**`record_fingerprint`** / **`record_fingerprint_here`** on success:

```json
{
  "ok": true,
  "command": "record_fingerprint",
  "label": "Cafe, WoH1",
  "x_m": 35.16,
  "y_m": 2.15,
  "bssids_heard": 7,
  "ap_rssi": { "Cafe, WoH1": -58.2, "EastPantry": -71.0 },
  "backend": "wpa_cli",
  "scans": 5,
  "db_path": "/root/.viam/module-data/fingerprints.sqlite",
  "recorded_at": "2026-06-05T14:30:00+00:00"
}
```

**`list_fingerprints`**:

```json
{
  "ok": true,
  "command": "list_fingerprints",
  "db_path": "/root/.viam/module-data/fingerprints.sqlite",
  "count": 3,
  "fingerprints": [
    {
      "label": "Cafe, WoH1",
      "x_m": 35.16,
      "y_m": 2.15,
      "ap_count": 7,
      "recorded_at": "2026-06-05T14:30:00+00:00",
      "scan_count": 5
    }
  ]
}
```

**`calibrate_path_loss`**:

```json
{
  "ok": true,
  "command": "calibrate_path_loss",
  "tx_power_dbm": -47.3,
  "path_loss_n": 3.21,
  "rmse_db": 5.4,
  "sample_count": 38,
  "fingerprint_count": 9,
  "per_ap_residuals_db": [
    { "ap_name": "WoLab", "mean_residual_db": -9.8, "sample_count": 6 },
    { "ap_name": "SoA1", "mean_residual_db": 1.2, "sample_count": 8 }
  ],
  "current": { "tx_power_dbm": -40.0, "path_loss_n": 2.5, "rmse_db": 14.2 },
  "applied": false,
  "db_path": "/root/.viam/module-data/fingerprints.sqlite"
}
```

`current.rmse_db` is how well your currently configured parameters explain the same data — if the fitted `rmse_db` is much lower, update your config. One AP with a large `mean_residual_db` while the rest sit near zero usually means that AP's configured coordinates (or BSSID) are wrong. A `warnings` array is included when the fit looks implausible (e.g. `path_loss_n` outside ~1.5–6).

**`delete_fingerprint`**: `{ "ok": true, "command": "delete_fingerprint", "label": "…", "deleted": true }`  
**`clear_fingerprints`**: `{ "ok": true, "command": "clear_fingerprints", "removed": 3, "db_path": "…" }`

Errors (unknown `ap_name`, missing fields, empty scan) raise an exception from `do_command` with a descriptive message.

#### Viam SDK (Python)

```python
from viam.components.sensor import Sensor
from viam.services.machine import Machine

async with Machine.create_from_address("<machine-address>", "<api-key>", "<api-key-id>") as machine:
    sensor = Sensor.from_name("wifi-position")
    result = await sensor.do_command({"command": "record_fingerprint", "ap_name": "Cafe, WoH1"})
    print(result)
```

#### Local wrapper equivalents

The same commands are available from **`test_scan_rssi.py`** without the Viam app:

```bash
sudo python3 test_scan_rssi.py --config examples/module_config_viam-5g.json \
  --record-fingerprint "Cafe, WoH1" --scan-mode thorough
sudo python3 test_scan_rssi.py --config examples/module_config_viam-5g.json \
  --record-fingerprint-here "Jane Smith's Desk" --at "12.0,5.5" --scan-mode thorough
sudo python3 test_scan_rssi.py --config examples/module_config_viam-5g.json \
  --scan-mode thorough --record-fingerprint-rssi "Matt Desk" --distance-to-ap "SoA1:8.3"
sudo python3 test_scan_rssi.py --config examples/module_config_viam-5g.json --list-fingerprints
sudo python3 test_scan_rssi.py --config examples/module_config_viam-5g.json --delete-fingerprint "Cafe, WoH1"
sudo python3 test_scan_rssi.py --config examples/module_config_viam-5g.json --clear-fingerprints
sudo python3 test_scan_rssi.py --config examples/module_config_viam-5g.json --set-device-z-m 1.2 --once
python3 test_scan_rssi.py --config examples/module_config_viam-5g.json --calibrate-path-loss
# Positioning auto-calibrates from fingerprints by default (first reading, then hourly):
sudo python3 test_scan_rssi.py --config examples/module_config_viam-5g.json --once
# nearest_fingerprint and fingerprint_rankings appear in --json output
sudo python3 test_scan_rssi.py --config examples/module_config_viam-5g.json --json --once
# Disable: --no-auto-calibrate-path-loss. Tune interval: --path-loss-calibration-interval 600
```

`--record-fingerprint-here` mirrors the `record_fingerprint_here` do_command: any label string,
with `--at "X,Y"` (or `"X,Y,Z"`) in the same meters frame as readings.

Default DB path: `<config-dir>/fingerprints.sqlite` (e.g. `examples/fingerprints.sqlite`). Override with **`--fingerprint-db`**. **`--set-device-z-m`** mirrors **`set_device_z_m`** for the local wrapper (use with **`--once`** or **`--interval`** in the same invocation).

### Positioning options

Component attributes that affect **`get_readings()`** (defaults shown):

| Attribute | Default | Role |
|-----------|---------|------|
| `fingerprint_max_blend` | `0.5` | Max fingerprint pull when blending (0 = geometry only, 1 = full pull at max confidence) |
| `min_anchors` | `3` | Minimum APs after RSSI filtering |
| `min_samples_per_ap` | auto | `2` when `scan_count >= 3`, else `1` |
| `max_rssi_delta_db` | `20` | Down-weight anchors much weaker than strongest (soft ramp, not a hard cutoff) |
| `min_rssi_dbm` | `-82` | Drop very weak anchors (below ~-80 dBm RSSI has almost no ranging value) |
| `weight_temperature` | `2.0` | Softens centroid RSSI weighting (see below) |
| `smoothing_alpha` | `1.0` | Temporal smoothing (`1` = no lag) |
| `max_position_step_m` | `0` | Cap movement per reading (`0` = off) |
| `scan_mode` | `fast` | `fast`, `thorough` (slower, steadier RSSI — use when calibrating), or `blocking` |

`weight_temperature` controls how sharply the weighted centroid favors the
strongest AP. `1.0` weights by raw received power, so the result snaps to
whichever AP is momentarily strongest and can jump several meters when that
ordering flips. Higher values flatten the weights so several nearby anchors
contribute, trading a little responsiveness for a much steadier position. The
default `2.0` is a good starting point; raise it if the position is still jumpy
while stationary, lower it toward `1.0` if it feels sluggish to follow you.

| `auto_calibrate_path_loss` | `true` | Periodically fit `tx_power_dbm` / `path_loss_n` from the fingerprint DB and apply in memory |
| `path_loss_calibration_interval_s` | `3600` | Seconds between automatic path-loss calibrations |

Other optional attributes: `interface`, `backend`, `scan_delay_s`, `strict_mac`, `tx_power_dbm`, `path_loss_n`, `fingerprint_db_path`, `fingerprint_k`, `fingerprint_min_common_aps`, `fingerprint_min_common_fraction`, `fingerprint_max_rms_db`.

Automatic calibration needs enough stored fingerprints (same minimum as `calibrate_path_loss`). Fits with implausible `tx_power_dbm` or `path_loss_n` are skipped. Applied values live until restart — copy fitted values into `tx_power_dbm` / `path_loss_n` in the config to make them permanent, or rely on auto-calibration to refresh them periodically.

### Continuous background scanning (mobile robots)

Blocking on `scan_count` WiFi scan passes per reading (often several seconds)
makes the reported position lag and smear across the path while the robot
moves. WiFi scans cannot run in parallel on one radio (the kernel serializes
them), but they can be **pipelined**: a daemon thread scans continuously into a
rolling, timestamped buffer, and `get_readings()` returns in milliseconds by
aggregating the freshest window, weighting **newer samples higher**
(exponential decay) — so the estimate tracks where the robot *is* rather than
the average of where it has been. Stationary, the window still smooths each
AP's RSSI in dB-space before triangulation, which steadies the position
without the lag of position-space smoothing.

**Background scanning is the default in the module** (the sensor is polled
repeatedly on a robot, which is exactly the continuous case). Set
`background_scan: false` to revert to blocking per-reading scans.

| Attribute | Default | Role |
|-----------|---------|------|
| `background_scan` | `true` | Continuous scanner thread; `false` = blocking per-reading scans |
| `background_scan_interval_s` | `0.5` | Pause between background scan passes |
| `rssi_window_s` | `8.0` | Sliding window of samples used per reading |
| `rssi_half_life_s` | `2.5` | Recency-weighting half-life; lower tracks motion faster, higher smooths more |

Notes:

- `scan_count` is ignored while background scanning is active; the window
  typically spans many more passes than a blocking reading would.
- Scan failures (e.g. a transiently busy interface) are retried with backoff
  and surfaced in the reading error only if the whole window is empty.
- Continuous scanning contends slightly more with normal traffic on the same
  radio; raise `background_scan_interval_s` to back it off, or set
  `background_scan: false`.
- Local wrapper: background scanning turns on automatically with `--interval`
  (one-shot runs use blocking scans). Disable with `--no-background-scan`;
  tune with `--background-scan-interval`, `--rssi-window`, `--rssi-half-life`:

```bash
# background scanning is automatic here
sudo python3 test_scan_rssi.py --config examples/module_config_viam-5g.json --interval 1

# old blocking behavior
sudo python3 test_scan_rssi.py --config examples/module_config_viam-5g.json \
  --interval 1 --no-background-scan
```

### Motion fusion (mobile robots)

A WiFi RSSI fix is a noisy, biased absolute position; a robot's own motion is
smooth and locally accurate but drifts. When you name one or more **optional**
motion sources, `get_readings()` fuses them with the WiFi fix in an adaptive
2D Kalman filter: it predicts from motion between scans and corrects with each
fix, so the position barely moves while the robot is stationary and tracks
quickly while driving. Fixes that disagree with the prediction by more than
`fusion_max_innovation_m` are rejected as outliers (after several consecutive
rejections the filter re-seeds, so it still recovers if the robot is moved).

All three sources are optional and independent — configure none (filter off,
behaves exactly as before), one, or several. Each is the **resource name** of a
component/service already on the machine:

| Attribute | Source | What it contributes |
|-----------|--------|---------------------|
| `movement_sensor` | A movement sensor (IMU, GPS, `wheeled-odometry`) | Speed from `get_linear_velocity()` → adapts smoothing (more responsive while moving) |
| `slam` | A SLAM service | Relative pose delta from `get_position()` → directional prediction between fixes |
| `base` | A base | `is_moving()` gate → freezes/heavily smooths when stopped, trusts motion when driving |

```json
{
  "movement_sensor": "imu",
  "slam": "slam-1",
  "base": "base",
  "motion_fusion": true
}
```

| Attribute | Default | Role |
|-----------|---------|------|
| `motion_fusion` | auto | Master switch; on by default when any source is named, set `false` to disable |
| `fusion_process_noise_m` | `0.5` | Baseline drift per reading while stationary (higher = follows fixes more) |
| `fusion_measurement_noise_m` | `3.0` | Assumed WiFi fix error; auto-tightened by anchor count and fingerprint confidence |
| `fusion_max_innovation_m` | `8.0` | Reject a fix that jumps more than this from the prediction (`0` disables gating) |
| `fusion_speed_scale` | `1.0` | How strongly motion speed loosens smoothing |
| `base_moving_speed_mps` | `0.5` | Assumed speed when only a base reports `is_moving` (no velocity source) |
| `slam_yaw_offset_deg` | `0.0` | Rotate the SLAM motion delta into the floor frame if the SLAM map is rotated |
| `slam_scale` | `1.0` | Scale correction if SLAM units don't match the floor plan |

Notes:

- SLAM gives the best results because it provides true directional motion. Its
  map frame may be rotated/scaled relative to your floor plan — use
  `slam_yaw_offset_deg` / `slam_scale` to align the **motion delta** (only
  relative motion is used, so the SLAM origin need not match the floor origin).
- A **base** has no odometry in the Viam API, so it acts only as a moving/stopped
  gate. For true wheel odometry, configure a `wheeled-odometry` **movement
  sensor** and name it under `movement_sensor`.
- When motion fusion is active it replaces the simpler `smoothing_alpha` /
  `max_position_step_m` filter; those still apply when no motion source is set.
- Each motion read is best-effort: if a source errors, it is logged and skipped
  for that reading rather than failing the position.

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

Then localize — fingerprints blend in automatically once the DB has entries (the `Method:` line shows `hybrid` when they contributed):

```bash
sudo python3 test_scan_rssi.py --config examples/module_config_viam-5g.json --interval 2
```

Tune the blend with `--fingerprint-max-blend` (0 = geometry only, 1 = full fingerprint pull at max confidence).

**Speed:** fingerprint blending does not add extra WiFi scans. With `--interval`, [background scanning](#continuous-background-scanning-mobile-robots) is on by default and each cycle is near-instant (it reads the rolling window). With `--no-background-scan` (or one-shot runs), each reading blocks on `scan_count` full scans (often **~2s each** on a Pi with `iw`, or longer if `iw` retries “device busy” then falls back to `wpa_cli`) — so `--scans 3` is often **~6–7s per cycle**, and `--interval 0.2` only sleeps 0.2s *after* that work finishes.

**`--scan-mode`** picks the scan strategy: `fast` (default; `wpa_cli` first, short poll waits), `thorough` (slower scans with delays between passes — use when recording fingerprints for maximum RSSI stability), or `blocking` (full blocking `iw` scan).

```bash
sudo python3 test_scan_rssi.py --scans 2 --interval 0.5
sudo python3 test_scan_rssi.py --record-fingerprint "Cafe, WoH1" --scan-mode thorough
```

Output includes `Cycle time: …s` so you can see actual duration. On the robot the equivalent attribute is `"scan_mode"` (default `"fast"`).

**Important:** AP-only fingerprints sit at the **AP's floor-plan coordinates** (discrete
points). The output line `Fingerprint: <name>` shows the nearest match; if `Method:` stays
`weighted_centroid`, the match was too poor to blend (loosen `--fingerprint-max-rms` or
record more fingerprints). Matching uses **relative** RSSI (strongest AP = 0 dB) and requires
several overlapping APs by default (`--fingerprint-min-common-aps 3`). A denser fingerprint
grid (e.g. `--record-fingerprint-here` every few meters) makes matches both more confident
and more precise.

On the robot, set `"fingerprint_db_path"` to a persistent path. See [Methods → `do_command()`](#do_command-fingerprint-calibration) for calibration commands.

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
chmod +x run.sh setup.sh build.sh
./setup.sh
viam module reload-local --part-id <part-id> \
  --model-name viam-labs:rssi-triangulation:wifi-position --name wifi-position
```

`meta.json` entrypoint is `run.sh`, which activates `.venv` and runs `src/main.py`.
`build.sh` packages `module.tar.gz` (includes `requirements.txt`, `pyproject.toml`, `src/`, `rssi_triangulation/`).

After changing module code, rebuild and redeploy:

```bash
./build.sh
viam module upload --version=0.0.2 --platform=linux/arm64 module.tar.gz
# or: viam module reload --part-id <part-id> --model-name viam-labs:rssi-triangulation:wifi-position --resource-name wifi-position
```

Paste your module config JSON into the component attributes in the Viam app.
