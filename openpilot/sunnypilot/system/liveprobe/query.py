"""Query the live probe and change whitelisted live params while driving.

  python -m openpilot.sunnypilot.system.liveprobe.query summary [seconds=20]
  python -m openpilot.sunnypilot.system.liveprobe.query table [seconds=20] [step=1]
  python -m openpilot.sunnypilot.system.liveprobe.query mark [seconds=180] [note...]
  python -m openpilot.sunnypilot.system.liveprobe.query get KEY | set KEY VALUE | undo | whitelist

Param writes go only to WHITELIST keys inside their range, each logged with the old value to
CHANGE_LOG; undo restores the last change that was not undone yet.
"""
import json
import os
import socket
import sys
import time
from datetime import UTC, datetime

import numpy as np

from openpilot.common.params import Params
from openpilot.sunnypilot.system.liveprobe.liveprobe import SOCKET_PATH

DATA_DIR = "/data/liveprobe"
CHANGE_LOG = os.path.join(DATA_DIR, "param_changes.jsonl")
MARK_DIR = os.path.join(DATA_DIR, "marks")

WHITELIST: dict[str, tuple[float, float]] = {
  "LaneCenteringWallClearance": (0.3, 1.0),
  "LaneCenteringWallInsideCap": (0.3, 1.5),
  "LaneCenteringOncomingClearance": (0.0, 0.6),
  "LaneCenteringOncomingCap": (0.0, 0.6),
  "LaneCenteringMinClearance": (0.0, 0.5),
  "LaneCenterOffset": (0.0, 0.3),
  "LaneCenteringMiddleBias": (-0.2, 0.2),
  "MaxLateralAccel": (3.0, 4.5),
  "SteerDriverAllowanceLow": (75, 250),
  "LagdMaxDelay": (0.3, 0.8),
  "LagdCityDelayBoost": (0.0, 0.5),
  "LagdToggleDelay": (0.1, 0.4),
  "IoniqHighwayCenterDamping": (0.0, 0.2),
  "RippleNotchQ": (0.0, 8.0),
  "ModelYawRateBlend": (0.0, 0.7),
  "LatHighwayKpScale": (0.3, 1.0),
  "ModelCrawlLatDeadband": (0.0, 0.1),
  "LatSplitPathHz": (0.0, 0.6),  # 0 = off; 0.35-0.6 = scene low-pass cutoff (split_path.py)
  # EPS limit test (2026-11-06): car-layer limits in the 18-55 km/h band, panda allows 1023 / 50
  "SteerHeadroomCan": (409, 1023),
  "SteerDeltaUpLow": (2, 50),
  "SteerDeltaDownLow": (3, 50),
}

CAUSES = {0: "none", 1: "right line", 2: "left line", 3: "right edge", 4: "left edge", 5: "right wall", 6: "left wall",
          7: "wall no-shift R", 8: "wall no-shift L", 9: "narrow"}
POSITIONS = {0: "unknown", 1: "single", 2: "right", 3: "middle", 4: "left"}

# (field, label, scale, format) for summary and table
SUMMARY = [
  ("v_kph", "speed km/h", 1, "{:.0f}"),
  ("lane_w", "lane width m", 1, "{:.2f}"),
  ("gap_line_l", "body to left line m", 1, "{:+.2f}"),
  ("gap_line_r", "body to right line m", 1, "{:+.2f}"),
  ("p_l", "left line prob", 1, "{:.2f}"),
  ("p_r", "right line prob", 1, "{:.2f}"),
  ("gap_wall_l", "body to left wall m", 1, "{:.2f}"),
  ("gap_wall_r", "body to right wall m", 1, "{:.2f}"),
  ("gap_edge_l", "body to left edge m", 1, "{:.2f}"),
  ("gap_edge_r", "body to right edge m", 1, "{:.2f}"),
  ("off", "offset m (+ right)", 1, "{:+.2f}"),
  ("off_target", "offset target m", 1, "{:+.2f}"),
  ("plan10", "plan @10 m (model frame)", 1, "{:+.2f}"),
  ("model_curv", "model curvature e-3", 1e3, "{:+.1f}"),
  ("des_curv", "desired curvature e-3", 1e3, "{:+.1f}"),
  ("act_curv", "actual curvature e-3", 1e3, "{:+.1f}"),
  ("tq_req", "torque request", 1, "{:+.2f}"),
  ("tq_out", "torque output", 1, "{:+.2f}"),
  ("drv_tq", "driver torque CAN", 1, "{:+.0f}"),
]
TABLE = [("v_kph", "kmh", 1, "{:3.0f}"), ("gap_wall_l", "wallL", 1, "{:5.2f}"), ("gap_line_l", "lineL", 1, "{:+5.2f}"),
         ("gap_line_r", "lineR", 1, "{:+5.2f}"), ("gap_wall_r", "wallR", 1, "{:5.2f}"), ("lane_w", "width", 1, "{:4.2f}"),
         ("off", "off", 1, "{:+5.2f}"), ("off_target", "tgt", 1, "{:+5.2f}"), ("des_curv", "dcur", 1e3, "{:+6.1f}"),
         ("act_curv", "acur", 1e3, "{:+6.1f}"), ("tq_out", "tq", 1, "{:+5.2f}"), ("drv_tq", "drv", 1, "{:+4.0f}")]


def fetch(seconds: float, path: str = SOCKET_PATH) -> list[dict]:
  s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
  s.settimeout(3.0)
  s.connect(path)
  s.sendall(f"rows {seconds}".encode())
  data = b""
  while chunk := s.recv(65536):
    data += chunk
  s.close()
  return json.loads(data)


def _vals(rows: list[dict], key: str) -> np.ndarray:
  return np.array([r[key] for r in rows if r.get(key) is not None], dtype=float)


def _mode(rows: list[dict], key: str, names: dict) -> str:
  v = [r[key] for r in rows if r.get(key) is not None]
  if not v:
    return "-"
  u, n = np.unique(v, return_counts=True)
  order = np.argsort(-n)
  return ", ".join(f"{names.get(int(u[i]), u[i])} {n[i] / len(v):.0%}" for i in order[:3])


def summary(rows: list[dict]) -> str:
  if not rows:
    return "no data (probe not running or not onroad)"
  span = rows[-1]["t"] - rows[0]["t"]
  lat = np.mean([bool(r.get("lat")) for r in rows])
  pressing = np.mean([bool(r.get("pressed")) for r in rows])
  out = [f"last {span:.0f} s, {len(rows)} rows, lateral active {lat:.0%}, driver pressing {pressing:.0%}"]
  for key, label, scale, fmt in SUMMARY:
    v = _vals(rows, key) * scale
    if v.size == 0:
      out.append(f"  {label}: -")
      continue
    seen = f" (seen {v.size / len(rows):.0%})" if v.size < len(rows) else ""
    out.append(f"  {label}: median {fmt.format(np.median(v))}, min {fmt.format(v.min())}, max {fmt.format(v.max())}{seen}")
  out.append(f"  lane position: {_mode(rows, 'lc_pos', POSITIONS)}")
  out.append(f"  limit cause: {_mode(rows, 'cause', CAUSES)}")
  bs = [(bool(r.get("bs_l")), bool(r.get("bs_r"))) for r in rows]
  out.append(f"  blind spot L/R: {np.mean([b[0] for b in bs]):.0%} / {np.mean([b[1] for b in bs]):.0%}")
  tq = _vals(rows, "tq_out")
  if tq.size:
    out.append(f"  torque output at >=1.5 (650 clip): {np.mean(np.abs(tq) >= 1.5):.0%}")
  alerts = sorted({r["alert"] for r in rows if r.get("alert")})
  if alerts:
    out.append(f"  alerts: {'; '.join(alerts)}")
  g = [r for r in rows if r.get("lat_deg") is not None]
  if g:
    out.append(f"  gps: {g[-1]['lat_deg']:.5f}, {g[-1]['lon_deg']:.5f}")
  return "\n".join(out)


def table(rows: list[dict], step: float = 1.0) -> str:
  if not rows:
    return "no data"
  t_end = rows[-1]["t"]
  head = "   t  " + " ".join(f"{lab:>6}" for _, lab, _, _ in TABLE) + "  pos/cause"
  lines = [head]
  t = rows[0]["t"]
  while t < t_end + 1e-6:
    w = [r for r in rows if t <= r["t"] < t + step]
    t += step
    if not w:
      continue
    cells = []
    for key, _, scale, fmt in TABLE:
      v = _vals(w, key) * scale
      cells.append(f"{fmt.format(np.median(v)):>6}" if v.size else f"{'-':>6}")
    pc = f"{POSITIONS.get(int(w[-1].get('lc_pos') or 0), '?')}/{CAUSES.get(int(w[-1].get('cause') or 0), '?')}"
    lines.append(f"{w[0]['t'] - t_end:+5.0f}s " + " ".join(cells) + "  " + pc)
  return "\n".join(lines)


def mark(rows: list[dict], note: str) -> str:
  os.makedirs(MARK_DIR, exist_ok=True)
  path = os.path.join(MARK_DIR, time.strftime("%Y%m%d-%H%M%S", time.gmtime()) + ".json")
  with open(path, "w") as f:
    json.dump({"note": note, "rows": rows}, f)
  return f"saved {len(rows)} rows to {path}"


def _typed(params: Params, key: str, value: float):
  old = params.get(key, return_default=True)
  return int(round(value)) if isinstance(old, int) and not isinstance(old, bool) else float(value), old


def set_param(params: Params, key: str, value: str, log_path: str = CHANGE_LOG) -> str:
  if key not in WHITELIST:
    return f"refused: {key} is not on the whitelist ({', '.join(WHITELIST)})"
  lo, hi = WHITELIST[key]
  try:
    v = float(value)
  except ValueError:
    return f"refused: {value!r} is not a number"
  if not lo <= v <= hi:
    return f"refused: {key} must be within {lo}..{hi}"
  new, old = _typed(params, key, v)
  params.put(key, new)
  os.makedirs(os.path.dirname(log_path), exist_ok=True)
  with open(log_path, "a") as f:
    f.write(json.dumps({"utc": datetime.now(UTC).timestamp(), "key": key, "old": old, "new": new}) + "\n")
  return f"{key}: {old} -> {new}"


def undo(params: Params, log_path: str = CHANGE_LOG) -> str:
  try:
    with open(log_path) as f:
      entries = [json.loads(line) for line in f if line.strip()]
  except FileNotFoundError:
    return "nothing to undo"
  stack: list[dict] = []
  for entry in entries:
    if entry.get("undo"):
      if stack:
        stack.pop()
    else:
      stack.append(entry)
  if not stack:
    return "nothing to undo"
  e = stack[-1]
  params.put(e["key"], e["old"])
  with open(log_path, "a") as f:
    f.write(json.dumps({"utc": datetime.now(UTC).timestamp(), "key": e["key"], "old": e["new"], "new": e["old"], "undo": True}) + "\n")
  return f"{e['key']}: {e['new']} -> {e['old']} (undone)"


def main(argv: list[str]) -> str:
  cmd = argv[0] if argv else "summary"
  if cmd == "summary":
    return summary(fetch(float(argv[1]) if len(argv) > 1 else 20.0))
  if cmd == "table":
    return table(fetch(float(argv[1]) if len(argv) > 1 else 20.0), float(argv[2]) if len(argv) > 2 else 1.0)
  if cmd == "mark":
    return mark(fetch(float(argv[1]) if len(argv) > 1 else 180.0), " ".join(argv[2:]))
  params = Params()
  if cmd == "get" and len(argv) == 2:
    return f"{argv[1]}: {params.get(argv[1], return_default=True)}"
  if cmd == "set" and len(argv) == 3:
    return set_param(params, argv[1], argv[2])
  if cmd == "undo":
    return undo(params)
  if cmd == "whitelist":
    return "\n".join(f"{k}: {lo}..{hi} (now {params.get(k, return_default=True)})" for k, (lo, hi) in WHITELIST.items())
  return __doc__


if __name__ == "__main__":
  print(main(sys.argv[1:]))
