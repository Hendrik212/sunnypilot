# openpilot MCP Servers

Two complementary MCP servers for analyzing openpilot drives with an AI assistant:

| Server | Language | Transport | Data source |
|---|---|---|---|
| **Cabana MCP** | C++ (Qt) | TCP | CAN bus signals via DBC |
| **Cereal MCP** | Python | stdio | Cereal log messages (rlog) |

Use both together: Cabana for raw CAN signal inspection, Cereal for high-level openpilot internals (controller state, model output, PID, actuators).

---

## Cabana MCP Server

### Build

```bash
# On the Linux build box
cd ~/openpilot
scons -j$(nproc) tools/cabana/cabana
```

### Run

```bash
DISPLAY=:99 tools/cabana/cabana \
  --data_dir /path/to/route_data \
  --mcp-server \
  --mcp-port 3002 &
```

- `--data_dir` must contain segment directories directly (e.g. `seg_0/`, `seg_1/`, ...) each holding a `rlog` or `rlog.zst` file.
- `--mcp-port` defaults to `3001`. Use a different port if occupied (e.g. `3002`).
- The server binds on all interfaces (`0.0.0.0`), so it's reachable from other machines on the network.
- DBC files are auto-loaded from `car_fingerprint_to_dbc.json` next to the binary. If the car isn't detected, load a DBC manually in the UI.

### MCP client config (TCP)

```json
{
  "mcpServers": {
    "cabana": {
      "type": "tcp",
      "host": "192.168.1.131",
      "port": 3002
    }
  }
}
```

### Tools

| Tool | Description |
|---|---|
| `decode_message` | Decode a raw hex CAN frame using the loaded DBC |
| `get_signal_values` | Time-series of CAN signals, resampled to a uniform grid, TOON tabular output |
| `search_signals` | Search signal names/descriptions across all loaded DBC messages |
| `export_data` | Export CAN data as CSV, JSON events, or DBC text |
| `analyze_signal_patterns` | Find oscillation, threshold, or rate-of-change passages in a CAN signal |

### Generating the DBC JSON (first-time setup)

```bash
cd ~/openpilot
python3 tools/cabana/dbc/generate_dbc_json.py \
  --out tools/cabana/dbc/car_fingerprint_to_dbc.json
```

Then place the JSON next to the `cabana` binary (or in `dbc/` relative to it).

---

## Cereal MCP Server

Reads openpilot rlog files and exposes the same signals visible in PlotJuggler's `tuning.xml` layout — lateral controller internals, model output, PID state, actuator deltas.

### Requirements

Run from inside the openpilot repo so imports resolve:

```bash
cd ~/openpilot
python3 tools/mcp/cereal_mcp.py
```

Python dependencies are the same as openpilot's existing tooling (`cereal`, `capnp`, `zstandard`).

### MCP client config (stdio)

```json
{
  "mcpServers": {
    "cereal": {
      "type": "stdio",
      "command": "python3",
      "args": ["/home/hendrik/openpilot/tools/mcp/cereal_mcp.py"],
      "env": {
        "CEREAL_DATA_DIR": "/home/hendrik/rlog_data"
      }
    }
  }
}
```

`CEREAL_DATA_DIR` is a convenience default — every tool also accepts a `data_dir` argument to override it per-call.

### Preparing route data

Copy rlog files from the comma device to the Linux box. Each segment needs its own subdirectory:

```
rlog_data/
  seg_50/rlog.zst
  seg_51/rlog.zst
  seg_52/rlog.zst
```

Transfer a range of segments from the device:

```bash
for seg in 50 51 52 53 54; do
  ssh hendrik@192.168.1.131 "mkdir -p ~/rlog_data/seg_$seg"
  ssh comma@192.168.1.100 "cat /data/media/0/realdata/<route_id>--$seg/rlog.zst" | \
    ssh hendrik@192.168.1.131 "cat > ~/rlog_data/seg_$seg/rlog.zst"
done
```

Find route IDs on the device:

```bash
ssh comma@192.168.1.100 "ls /data/media/0/realdata/ | grep -v 'boot\|crash\|params' | sed 's/--[0-9]*$//' | sort -u"
```

### Tools

#### `list_segments`

List all rlog files found under a directory.

```json
{ "data_dir": "/home/hendrik/rlog_data" }
```

#### `get_lateral_data`

Returns all lateral tuning signals aligned on a uniform time grid (equivalent to the **Lateral** + **Lateral Debug** tabs in `tools/plotjuggler/layouts/tuning.xml`).

```json
{
  "data_dir": "/home/hendrik/rlog_data",
  "time_range": "0-120",
  "sample_rate": 0.1
}
```

`time_range` is in seconds relative to the first message in the first segment. Omit or use `"all"` for the full duration. `sample_rate` defaults to `0.1` (10 Hz); use `0.02` for full 50 Hz resolution.

**Signals in the output:**

| Signal | Source message | Description |
|---|---|---|
| `vEgo` | `carState` | Vehicle speed (m/s) |
| `steeringAngleDeg` | `carState` | Steering wheel angle (deg) |
| `steeringTorque` | `carState` | Driver torque (native units) |
| `steeringTorqueEps` | `carState` | EPS torque (native units) |
| `steeringPressed` | `carState` | Human override active |
| `enabled` | `selfdriveState` | openpilot engaged |
| `curvature` | `controlsState` | Vehicle model curvature (1/m) |
| `desiredCurvature` | `controlsState` | Controller target curvature (1/m) |
| `modelDesiredCurvature` | `modelV2` | Model output curvature (1/m) |
| `curvatureError` | derived | `desiredCurvature - curvature` |
| `pidP` / `pidI` / `pidF` | `controlsState` | PID controller components |
| `pidSaturated` | `controlsState` | PID saturation flag |
| `desiredTorque` | `carControl` | Requested actuator torque |
| `outputTorque` | `carOutput` | Actual output torque (after rate limiting) |
| `desiredSteeringAngle` | `carControl` | Requested steering angle (deg) |
| `outputSteeringAngle` | `carOutput` | Actual output steering angle (deg) |
| `yawRate` | `liveLocationKalman` | Calibrated yaw rate (rad/s) |

Output is in TOON tabular format — a compact CSV-like encoding optimized for LLM context.

#### `get_signal_values`

Extract arbitrary cereal fields by dot-path. Useful for diving into specific sub-fields not covered by `get_lateral_data`.

```json
{
  "signals": [
    "carState.steeringAngleDeg",
    "controlsState.lateralControlState.pidState.p",
    "modelV2.action.desiredCurvature",
    "carState.vEgo"
  ],
  "data_dir": "/home/hendrik/rlog_data",
  "time_range": "60-180",
  "sample_rate": 0.1
}
```

Array indices are supported: `carControl.orientationNED.2` (yaw component).

#### `analyze_lateral`

Find ping-pong / oscillation passages. Filters to cleanly engaged samples only — `selfdriveState.enabled == true`, `steeringPressed == false`, and a 5-second cooldown after any override or disengage event (matching the `engage_delay` used in the plotjuggler tuning layout).

For each passage the tool reports peak curvature error, mean speed, and a **diagnosis hint** based on the Pearson correlation between `controlsState.desiredCurvature` and `modelV2.action.desiredCurvature`:

- **Correlation > 0.7** → model-driven: the controller faithfully follows the model, so the model is the source of oscillation
- **Correlation < 0.3** → controls-driven: the controller diverges from the model, amplifying on its own
- **In between** → mixed contribution

```json
{
  "data_dir": "/home/hendrik/rlog_data",
  "speed_min": 22,
  "threshold": 0.00005,
  "window": 3.0,
  "min_duration": 2.0
}
```

| Parameter | Default | Description |
|---|---|---|
| `speed_min` | `0` | Minimum speed in m/s (e.g. `22` ≈ 80 km/h for highway only) |
| `threshold` | `0.0005` | Rolling stddev threshold in 1/m. Lower = more sensitive. Start at `0.00005` to find subtle oscillations. |
| `window` | `3.0` | Rolling window size in seconds |
| `min_duration` | `2.0` | Minimum passage duration in seconds |
| `time_range` | `"all"` | Restrict to a time window, e.g. `"60-300"` |

---

## Using both servers together

A typical diagnostic session for lateral ping-pong:

1. **List available data**
   - `cereal/list_segments` to confirm rlog files are present

2. **Find oscillation passages**
   - `cereal/analyze_lateral` with `speed_min=22` (highway only)
   - Note the `start`/`end` timestamps of interesting passages and the `diagnosis_hint`

3. **Inspect the curvature triangle**
   - `cereal/get_lateral_data` with `time_range="<start>-<end>"` and `sample_rate=0.02`
   - Compare `modelDesiredCurvature`, `desiredCurvature`, and `curvature` to see where the deviation originates

4. **Check actuator rate limiting**
   - Look at `desiredTorque` vs `outputTorque` and `desiredSteeringAngle` vs `outputSteeringAngle`
   - A persistent gap indicates the rate limiter is clipping the controller's commands

5. **Cross-reference with raw CAN**
   - `cabana/get_signal_values` for the same time window on `STEERING_ANGLE`, wheel speeds, etc.
   - Useful to verify the cereal data against ground-truth CAN frames
