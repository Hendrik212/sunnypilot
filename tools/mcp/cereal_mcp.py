#!/usr/bin/env python3
"""
Cereal MCP Server — exposes openpilot cereal log data via MCP (stdio transport).

Tools:
  list_segments        — list available rlog segments under a data directory
  get_lateral_data     — time-series of all lateral tuning signals (tuning.xml equivalent)
  get_signal_values    — arbitrary cereal field time-series, multiple fields aligned
  analyze_lateral      — find ping-pong / oscillation passages in lateral error

Usage (stdio MCP, connect from MCP client):
  python3 tools/mcp/cereal_mcp.py

Environment:
  CEREAL_DATA_DIR   — path to segment directory (dirs containing rlog files)
                      can also be set per-call via the data_dir tool argument
"""

import json
import math
import os
import sys
import glob
import traceback
from typing import Any

# ---------------------------------------------------------------------------
# Bootstrap: ensure we can import openpilot modules
# ---------------------------------------------------------------------------
BASEDIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
if BASEDIR not in sys.path:
    sys.path.insert(0, BASEDIR)

# ---------------------------------------------------------------------------
# Lazy imports (only when a tool is called, so startup is fast)
# ---------------------------------------------------------------------------
_lr_module = None


def _get_logreader():
    global _lr_module
    if _lr_module is None:
        from openpilot.tools.lib.logreader import LogReader, _LogFileReader, ReadMode
        _lr_module = (LogReader, _LogFileReader, ReadMode)
    return _lr_module


# ---------------------------------------------------------------------------
# TOON-lite: compact tabular encoding for LLM output
# ---------------------------------------------------------------------------

def _fmt(v):
    """Format a scalar for tabular output."""
    if v is None or (isinstance(v, float) and (math.isnan(v) or math.isinf(v))):
        return "null"
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, float):
        # round to 6 sig figs to avoid noise
        s = f"{v:.6g}"
        return s
    if isinstance(v, int):
        return str(v)
    return json.dumps(str(v))


def _toon_tabular(rows: list[dict], fields: list[str]) -> str:
    """Encode a list of dicts as TOON tabular format."""
    n = len(rows)
    header = f"[{n}]{{{','.join(fields)}}}:"
    lines = [header]
    for row in rows:
        lines.append("  " + ",".join(_fmt(row.get(f)) for f in fields))
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Segment discovery
# ---------------------------------------------------------------------------

def _find_rlogs(data_dir: str) -> list[str]:
    """Return sorted list of rlog paths under data_dir."""
    patterns = [
        os.path.join(data_dir, "*/rlog"),
        os.path.join(data_dir, "*/rlog.bz2"),
        os.path.join(data_dir, "*/rlog.zst"),
        os.path.join(data_dir, "rlog"),
        os.path.join(data_dir, "rlog.bz2"),
        os.path.join(data_dir, "rlog.zst"),
    ]
    found = []
    for pat in patterns:
        found.extend(glob.glob(pat))
    return sorted(set(found))


# ---------------------------------------------------------------------------
# Field extraction helpers
# ---------------------------------------------------------------------------

# Mapping from short alias → (message_type, accessor_fn)
# accessor_fn(msg) → float or None
_LATERAL_FIELDS: dict[str, tuple[str, Any]] = {
    "ts":                    ("__time__", None),
    "vEgo":                  ("carState",       lambda m: m.vEgo),
    "steeringAngleDeg":      ("carState",       lambda m: m.steeringAngleDeg),
    "steeringTorque":        ("carState",       lambda m: m.steeringTorque),
    "steeringTorqueEps":     ("carState",       lambda m: m.steeringTorqueEps),
    "steeringPressed":       ("carState",       lambda m: float(m.steeringPressed)),
    "enabled":               ("selfdriveState", lambda m: float(m.enabled)),
    "curvature":             ("controlsState",  lambda m: m.curvature),
    "desiredCurvature":      ("controlsState",  lambda m: m.desiredCurvature),
    "lateralError":          ("controlsState",  lambda m: _pid_field(m, "p")),  # placeholder, overridden below
    "pidP":                  ("controlsState",  lambda m: _pid_field(m, "p")),
    "pidI":                  ("controlsState",  lambda m: _pid_field(m, "i")),
    "pidF":                  ("controlsState",  lambda m: _pid_field(m, "f")),
    "pidSaturated":          ("controlsState",  lambda m: float(_pid_saturated(m))),
    "modelDesiredCurvature": ("modelV2",        lambda m: m.action.desiredCurvature),
    "desiredTorque":         ("carControl",     lambda m: _actuator_torque(m)),
    "desiredSteeringAngle":  ("carControl",     lambda m: _actuator_angle(m)),
    "outputTorque":          ("carOutput",      lambda m: m.actuatorsOutput.torque),
    "outputSteeringAngle":   ("carOutput",      lambda m: m.actuatorsOutput.steeringAngleDeg),
    "yawRate":               ("liveLocationKalman", lambda m: _yaw_rate(m)),
}

# Derived curvature error: desiredCurvature - curvature (from controlsState)
# We compute this in post-processing, not from a single message.


def _pid_field(cs, field: str):
    """Extract PID sub-field from controlsState.lateralControlState."""
    try:
        state = cs.lateralControlState.which()
        sub = getattr(cs.lateralControlState, state)
        return float(getattr(sub, field, 0.0))
    except Exception:
        return None


def _pid_saturated(cs) -> bool:
    try:
        state = cs.lateralControlState.which()
        sub = getattr(cs.lateralControlState, state)
        return bool(getattr(sub, "saturated", False))
    except Exception:
        return False


def _actuator_torque(cc):
    try:
        return float(cc.actuators.torque)
    except Exception:
        return None


def _actuator_angle(cc):
    try:
        return float(cc.actuators.steeringAngleDeg)
    except Exception:
        return None


def _yaw_rate(llk):
    """Extract yaw rate (z component) from liveLocationKalman.angularVelocityCalibrated.value."""
    try:
        v = llk.angularVelocityCalibrated.value
        return float(v[2])
    except Exception:
        try:
            v = llk.angularVelocityDevice.value
            return float(v[2])
        except Exception:
            return None


# ---------------------------------------------------------------------------
# Core: read rlog files and extract time-series
# ---------------------------------------------------------------------------

def _read_segments(rlog_paths: list[str], wanted_types: set[str]) -> dict[str, list[tuple[float, Any]]]:
    """
    Read rlog files and return {msg_type: [(t_seconds, msg_payload), ...]} for wanted types.
    t_seconds is relative to the first message in the first segment.
    """
    _, _LogFileReader, _ = _get_logreader()

    result: dict[str, list] = {t: [] for t in wanted_types}
    t0 = None

    for path in rlog_paths:
        try:
            lr = _LogFileReader(path)
        except Exception as e:
            continue
        for evt in lr:
            try:
                mtype = evt.which()
            except Exception:
                continue
            if mtype not in wanted_types:
                continue
            t = evt.logMonoTime / 1e9
            if t0 is None:
                t0 = t
            result[mtype].append((t - t0, getattr(evt, mtype)))

    return result


def _extract_lateral(rlog_paths: list[str], time_range: tuple[float, float] | None,
                     fields: list[str], sample_rate: float) -> list[dict]:
    """
    Extract lateral fields from rlogs, resampled to a uniform grid.
    Returns list of row dicts with ts + requested fields.
    """
    # Determine which message types we need
    needed_types = set()
    for f in fields:
        if f in _LATERAL_FIELDS:
            mtype, _ = _LATERAL_FIELDS[f]
            if mtype != "__time__":
                needed_types.add(mtype)

    raw = _read_segments(rlog_paths, needed_types)

    # Find overall time bounds
    all_times = []
    for series in raw.values():
        if series:
            all_times.extend(t for t, _ in series)
    if not all_times:
        return []

    t_min = min(all_times)
    t_max = max(all_times)
    if time_range:
        t_min = max(t_min, time_range[0])
        t_max = min(t_max, time_range[1])

    if t_max <= t_min:
        return []

    # Build per-type sorted arrays for nearest-neighbor lookup
    sorted_series: dict[str, list] = {}
    for mtype, series in raw.items():
        sorted_series[mtype] = sorted(series, key=lambda x: x[0])

    def nearest(mtype: str, t: float, accessor):
        series = sorted_series.get(mtype, [])
        if not series:
            return None
        # binary search
        lo, hi = 0, len(series) - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if series[mid][0] < t:
                lo = mid + 1
            else:
                hi = mid
        # check lo-1 and lo
        best_i = lo
        if lo > 0 and abs(series[lo - 1][0] - t) < abs(series[lo][0] - t):
            best_i = lo - 1
        try:
            return accessor(series[best_i][1])
        except Exception:
            return None

    # Generate uniform grid
    num_steps = int((t_max - t_min) / sample_rate) + 1
    rows = []
    for i in range(num_steps):
        t = t_min + i * sample_rate
        if t > t_max + sample_rate / 2:
            break
        row = {"ts": round(t, 3)}
        for f in fields:
            if f == "ts":
                continue
            if f not in _LATERAL_FIELDS:
                row[f] = None
                continue
            mtype, accessor = _LATERAL_FIELDS[f]
            if mtype == "__time__":
                continue
            row[f] = nearest(mtype, t, accessor)
        rows.append(row)

    return rows


# ---------------------------------------------------------------------------
# Tool implementations
# ---------------------------------------------------------------------------

def tool_list_segments(args: dict) -> dict:
    data_dir = args.get("data_dir") or os.environ.get("CEREAL_DATA_DIR", "")
    if not data_dir:
        return {"error": "data_dir is required (or set CEREAL_DATA_DIR env var)"}
    if not os.path.isdir(data_dir):
        return {"error": f"directory not found: {data_dir}"}

    rlogs = _find_rlogs(data_dir)
    segments = []
    for path in rlogs:
        seg_dir = os.path.dirname(path) if os.path.basename(path).startswith("rlog") else path
        seg_name = os.path.relpath(seg_dir, data_dir) if seg_dir != data_dir else os.path.basename(path)
        size_mb = os.path.getsize(path) / 1e6
        segments.append({"path": path, "segment": seg_name, "size_mb": round(size_mb, 2)})

    return {
        "data_dir": data_dir,
        "segment_count": len(segments),
        "segments": segments,
    }


def tool_get_lateral_data(args: dict) -> str:
    """Return all lateral tuning signals aligned on a uniform time grid."""
    data_dir = args.get("data_dir") or os.environ.get("CEREAL_DATA_DIR", "")
    if not data_dir:
        return "error: data_dir is required"

    rlog_paths = _find_rlogs(data_dir)
    if not rlog_paths:
        return f"error: no rlog files found in {data_dir}"

    # Parse time_range
    time_range = _parse_time_range(args.get("time_range", "all"))
    sample_rate = float(args.get("sample_rate", 0.1))  # default 10 Hz

    # All lateral fields (excluding ts which is added automatically)
    fields = [
        "ts",
        "vEgo",
        "steeringAngleDeg",
        "steeringTorque",
        "steeringTorqueEps",
        "steeringPressed",
        "enabled",
        "curvature",
        "desiredCurvature",
        "modelDesiredCurvature",
        "pidP",
        "pidI",
        "pidF",
        "pidSaturated",
        "desiredTorque",
        "desiredSteeringAngle",
        "outputTorque",
        "outputSteeringAngle",
        "yawRate",
    ]

    rows = _extract_lateral(rlog_paths, time_range, fields, sample_rate)
    if not rows:
        return "no data found in the requested time range"

    # Add derived curvatureError
    for row in rows:
        dc = row.get("desiredCurvature")
        c = row.get("curvature")
        row["curvatureError"] = round(dc - c, 6) if dc is not None and c is not None else None

    all_fields = fields + ["curvatureError"]
    return _toon_tabular(rows, all_fields)


def tool_get_signal_values(args: dict) -> str:
    """
    Extract arbitrary cereal fields by dot-path, e.g.:
      "carState.steeringAngleDeg", "controlsState.curvature", "modelV2.action.desiredCurvature"
    Returns aligned time-series in TOON tabular format.
    """
    data_dir = args.get("data_dir") or os.environ.get("CEREAL_DATA_DIR", "")
    if not data_dir:
        return "error: data_dir is required"

    signal_paths: list[str] = args.get("signals", [])
    if not signal_paths:
        return "error: signals list is required"

    rlog_paths = _find_rlogs(data_dir)
    if not rlog_paths:
        return f"error: no rlog files found in {data_dir}"

    time_range = _parse_time_range(args.get("time_range", "all"))
    sample_rate = float(args.get("sample_rate", 0.1))

    # Parse signal paths → (msg_type, path_parts, alias)
    parsed = []
    needed_types = set()
    for sp in signal_paths:
        parts = sp.split(".")
        if len(parts) < 2:
            return f"error: signal '{sp}' must be msg_type.field[.subfield...]"
        mtype = parts[0]
        path_parts = parts[1:]
        alias = sp.replace(".", "_")
        parsed.append((mtype, path_parts, alias))
        needed_types.add(mtype)

    raw = _read_segments(rlog_paths, needed_types)

    # Time bounds
    all_times = []
    for series in raw.values():
        all_times.extend(t for t, _ in series)
    if not all_times:
        return "no data found"

    t_min = min(all_times)
    t_max = max(all_times)
    if time_range:
        t_min = max(t_min, time_range[0])
        t_max = min(t_max, time_range[1])

    sorted_series = {mtype: sorted(s, key=lambda x: x[0]) for mtype, s in raw.items()}

    def get_nested(obj, parts):
        for p in parts:
            if p.isdigit():
                obj = obj[int(p)]
            else:
                obj = getattr(obj, p)
        return float(obj)

    def nearest_val(mtype, path_parts, t):
        series = sorted_series.get(mtype, [])
        if not series:
            return None
        lo, hi = 0, len(series) - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if series[mid][0] < t:
                lo = mid + 1
            else:
                hi = mid
        best_i = lo
        if lo > 0 and abs(series[lo - 1][0] - t) < abs(series[lo][0] - t):
            best_i = lo - 1
        try:
            return get_nested(series[best_i][1], path_parts)
        except Exception:
            return None

    num_steps = int((t_max - t_min) / sample_rate) + 1
    fields = ["ts"] + [alias for _, _, alias in parsed]
    rows = []
    for i in range(num_steps):
        t = t_min + i * sample_rate
        if t > t_max + sample_rate / 2:
            break
        row = {"ts": round(t, 3)}
        for mtype, path_parts, alias in parsed:
            row[alias] = nearest_val(mtype, path_parts, t)
        rows.append(row)

    return _toon_tabular(rows, fields)


def tool_analyze_lateral(args: dict) -> dict:
    """
    Find passages where the lateral controller shows signs of ping-pong oscillation.
    Computes curvatureError = desiredCurvature - curvature from controlsState,
    then finds windows where the rolling stddev exceeds threshold.

    Also returns a passage-level summary with:
      - curvatureError oscillation amplitude
      - correlation between desiredCurvature and modelDesiredCurvature
        (to distinguish controls vs model issue)
    """
    data_dir = args.get("data_dir") or os.environ.get("CEREAL_DATA_DIR", "")
    if not data_dir:
        return {"error": "data_dir is required"}

    rlog_paths = _find_rlogs(data_dir)
    if not rlog_paths:
        return {"error": f"no rlog files found in {data_dir}"}

    time_range = _parse_time_range(args.get("time_range", "all"))
    window = float(args.get("window", 3.0))          # rolling window seconds
    threshold = float(args.get("threshold", 0.0005)) # curvatureError stddev threshold (1/m)
    min_duration = float(args.get("min_duration", 2.0))
    speed_min = float(args.get("speed_min", 0.0))    # only flag when speed >= this (m/s)

    # Extract at high resolution (50Hz) for accurate stddev
    sample_rate = 0.02
    engage_delay = 5.0  # seconds cooldown after steeringPressed or disengage, matching tuning.xml
    fields = ["ts", "vEgo", "enabled", "steeringPressed", "curvature", "desiredCurvature",
              "modelDesiredCurvature", "steeringAngleDeg", "desiredTorque", "outputTorque"]
    rows = _extract_lateral(rlog_paths, time_range, fields, sample_rate)
    if not rows:
        return {"error": "no data found"}

    # Filter to cleanly engaged samples: engaged, not steering-pressed, and at least
    # engage_delay seconds since the last bad event (steeringPressed or disengage).
    # This matches the engage_delay=5 cooldown used in tuning.xml custom equations.
    last_bad_time = -engage_delay
    clean_rows = []
    for r in rows:
        t = r.get("ts", 0)
        enabled = r.get("enabled") == 1.0
        pressed = r.get("steeringPressed") == 1.0
        speed_ok = (r.get("vEgo") or 0) >= speed_min
        if pressed or not enabled:
            last_bad_time = t
        if enabled and not pressed and speed_ok and (t - last_bad_time) >= engage_delay:
            clean_rows.append(r)

    engaged = clean_rows
    if not engaged:
        return {"error": "no cleanly engaged driving data found (engaged + no steeringPressed + "
                         f"{engage_delay}s cooldown + speed >= {speed_min} m/s)"}

    # Compute curvature error per sample
    for r in engaged:
        dc = r.get("desiredCurvature")
        c = r.get("curvature")
        r["curvError"] = (dc - c) if dc is not None and c is not None else 0.0

    # Rolling stddev over window
    half = int(window / sample_rate / 2)
    n = len(engaged)
    flags = []
    for i in range(n):
        lo = max(0, i - half)
        hi = min(n, i + half + 1)
        vals = [engaged[j]["curvError"] for j in range(lo, hi) if engaged[j]["curvError"] is not None]
        if len(vals) < 3:
            flags.append(False)
            continue
        mean = sum(vals) / len(vals)
        variance = sum((v - mean) ** 2 for v in vals) / len(vals)
        stddev = math.sqrt(variance)
        flags.append(stddev > threshold)

    # Merge consecutive flagged regions
    passages = []
    in_passage = False
    p_start = None
    for i, flagged in enumerate(flags):
        if flagged and not in_passage:
            in_passage = True
            p_start = i
        elif not flagged and in_passage:
            in_passage = False
            passages.append((p_start, i - 1))
    if in_passage:
        passages.append((p_start, n - 1))

    # Filter by min_duration and build result
    results = []
    for (pi, pj) in passages:
        t_start = engaged[pi]["ts"]
        t_end = engaged[pj]["ts"]
        duration = t_end - t_start
        if duration < min_duration:
            continue

        passage_rows = engaged[pi:pj + 1]
        errors = [r["curvError"] for r in passage_rows if r["curvError"] is not None]
        speeds = [r["vEgo"] for r in passage_rows if r.get("vEgo") is not None]
        desired = [r.get("desiredCurvature") for r in passage_rows if r.get("desiredCurvature") is not None]
        model = [r.get("modelDesiredCurvature") for r in passage_rows if r.get("modelDesiredCurvature") is not None]

        peak_error = max(abs(e) for e in errors) if errors else 0
        mean_speed = sum(speeds) / len(speeds) if speeds else 0

        # Correlation between controller desiredCurvature and model desiredCurvature
        # High correlation → model is driving the oscillation
        # Low correlation → controls amplifying
        corr = _pearson(desired, model) if len(desired) > 3 and len(model) > 3 else None

        results.append({
            "start": round(t_start, 2),
            "end": round(t_end, 2),
            "duration": round(duration, 2),
            "peak_curvature_error": round(peak_error, 6),
            "mean_speed_ms": round(mean_speed, 1),
            "mean_speed_kmh": round(mean_speed * 3.6, 1),
            "model_controls_correlation": round(corr, 3) if corr is not None else None,
            "diagnosis_hint": _diagnose(corr, peak_error),
        })

    return {
        "total_engaged_seconds": round(len(engaged) * sample_rate, 1),
        "threshold_used": threshold,
        "window_seconds": window,
        "passages_found": len(results),
        "passages": results,
    }


def _diagnose(corr: float | None, peak_error: float) -> str:
    if corr is None:
        return "insufficient data"
    if corr > 0.7:
        return "likely model-driven: controller desiredCurvature tracks model closely → model is oscillating"
    elif corr < 0.3:
        return "likely controls-driven: controller diverges from model → control loop amplifying"
    else:
        return "mixed: partial model + control contribution"


def _pearson(xs: list, ys: list) -> float | None:
    n = min(len(xs), len(ys))
    if n < 4:
        return None
    xs, ys = xs[:n], ys[:n]
    mx = sum(xs) / n
    my = sum(ys) / n
    num = sum((xs[i] - mx) * (ys[i] - my) for i in range(n))
    dx = math.sqrt(sum((x - mx) ** 2 for x in xs))
    dy = math.sqrt(sum((y - my) ** 2 for y in ys))
    if dx == 0 or dy == 0:
        return None
    return num / (dx * dy)


def _parse_time_range(s: str) -> tuple[float, float] | None:
    if not s or s.strip().lower() == "all":
        return None
    try:
        parts = s.split("-")
        return float(parts[0]), float(parts[1])
    except Exception:
        return None


# ---------------------------------------------------------------------------
# MCP stdio protocol
# ---------------------------------------------------------------------------

TOOLS = [
    {
        "name": "list_segments",
        "description": "List available rlog segments under a data directory.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "data_dir": {
                    "type": "string",
                    "description": "Path to directory containing segment subdirectories with rlog files."
                }
            },
        },
    },
    {
        "name": "get_lateral_data",
        "description": (
            "Return all lateral tuning signals as an aligned time-series (equivalent to the "
            "Lateral + Lateral Debug tabs in plotjuggler/layouts/tuning.xml). Signals include: "
            "vEgo, steeringAngleDeg, steeringTorque/Eps, curvature (vehicle model), "
            "desiredCurvature (controller), modelDesiredCurvature (model output), "
            "curvatureError (desired-actual), PID p/i/f/saturated, actuator torque vs output torque, "
            "desired vs output steeringAngleDeg, yawRate, enabled, steeringPressed. "
            "Output is TOON tabular format (compact CSV-style rows)."
        ),
        "inputSchema": {
            "type": "object",
            "properties": {
                "data_dir": {
                    "type": "string",
                    "description": "Path to directory containing segment subdirectories with rlog files."
                },
                "time_range": {
                    "type": "string",
                    "description": "Time range in seconds, e.g. '60-300' or 'all' (default: 'all')."
                },
                "sample_rate": {
                    "type": "number",
                    "description": "Seconds per sample (default: 0.1 = 10 Hz). Use 0.02 for full 50Hz."
                },
            },
        },
    },
    {
        "name": "get_signal_values",
        "description": (
            "Extract arbitrary cereal message fields by dot-path and return aligned time-series. "
            "Example signals: ['carState.steeringAngleDeg', 'controlsState.curvature', "
            "'modelV2.action.desiredCurvature', 'carState.vEgo', 'controlsState.lateralControlState.pidState.p']. "
            "Output is TOON tabular format."
        ),
        "inputSchema": {
            "type": "object",
            "required": ["signals"],
            "properties": {
                "signals": {
                    "type": "array",
                    "items": {"type": "string"},
                    "description": "List of dot-paths like 'msgType.field.subfield'."
                },
                "data_dir": {"type": "string"},
                "time_range": {"type": "string", "description": "e.g. '60-180' or 'all'"},
                "sample_rate": {"type": "number", "description": "seconds per sample (default 0.1)"},
            },
        },
    },
    {
        "name": "analyze_lateral",
        "description": (
            "Find passages where the lateral controller shows oscillation (ping-pong). "
            "Uses rolling stddev of curvatureError (desiredCurvature - vehicleModelCurvature). "
            "For each passage, reports duration, peak error, mean speed, and a diagnosis hint: "
            "whether oscillation is model-driven or controls-driven (based on correlation between "
            "controller desiredCurvature and modelV2 desiredCurvature)."
        ),
        "inputSchema": {
            "type": "object",
            "properties": {
                "data_dir": {"type": "string"},
                "time_range": {"type": "string", "description": "e.g. '0-600' or 'all'"},
                "window": {"type": "number", "description": "Rolling window in seconds (default 3.0)"},
                "threshold": {
                    "type": "number",
                    "description": "Curvature error stddev threshold in 1/m (default 0.0005). "
                                   "Lower = more sensitive."
                },
                "min_duration": {"type": "number", "description": "Minimum passage duration seconds (default 2.0)"},
                "speed_min": {"type": "number", "description": "Only analyze segments above this speed in m/s (default 0 = all)"},
            },
        },
    },
]


def handle_request(req: dict) -> dict | None:
    method = req.get("method", "")
    req_id = req.get("id")
    params = req.get("params", {})

    def ok(result):
        if req_id is None:
            return None
        return {"jsonrpc": "2.0", "id": req_id, "result": result}

    def err(code, msg):
        if req_id is None:
            return None
        return {"jsonrpc": "2.0", "id": req_id,
                "error": {"code": code, "message": msg}}

    if method == "initialize":
        return ok({
            "protocolVersion": "2024-11-05",
            "capabilities": {"tools": {}},
            "serverInfo": {"name": "cereal-mcp", "version": "1.0.0"},
        })

    if method == "notifications/initialized":
        return None

    if method == "tools/list":
        return ok({"tools": TOOLS})

    if method == "tools/call":
        tool_name = params.get("name", "")
        tool_args = params.get("arguments", {})
        try:
            if tool_name == "list_segments":
                result = tool_list_segments(tool_args)
                text = json.dumps(result, indent=2)
            elif tool_name == "get_lateral_data":
                text = tool_get_lateral_data(tool_args)
            elif tool_name == "get_signal_values":
                text = tool_get_signal_values(tool_args)
            elif tool_name == "analyze_lateral":
                result = tool_analyze_lateral(tool_args)
                text = json.dumps(result, indent=2)
            else:
                return err(-32601, f"Unknown tool: {tool_name}")

            return ok({
                "content": [{"type": "text", "text": text}],
                "isError": False,
            })
        except Exception as e:
            tb = traceback.format_exc()
            return ok({
                "content": [{"type": "text", "text": f"Error: {e}\n{tb}"}],
                "isError": True,
            })

    if method == "resources/list":
        return ok({"resources": []})

    # Unknown method — return error for requests, ignore for notifications
    if req_id is not None:
        return err(-32601, f"Method not found: {method}")
    return None


def main():
    # Read line-delimited JSON from stdin, write to stdout
    import io
    stdin = io.TextIOWrapper(sys.stdin.buffer, encoding="utf-8")
    stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", line_buffering=True)

    for line in stdin:
        line = line.strip()
        if not line:
            continue
        try:
            req = json.loads(line)
        except json.JSONDecodeError as e:
            resp = {"jsonrpc": "2.0", "id": None,
                    "error": {"code": -32700, "message": f"Parse error: {e}"}}
            stdout.write(json.dumps(resp) + "\n")
            continue

        resp = handle_request(req)
        if resp is not None:
            stdout.write(json.dumps(resp) + "\n")


if __name__ == "__main__":
    main()
