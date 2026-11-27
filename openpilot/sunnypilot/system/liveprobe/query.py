"""Query the live probe and change whitelisted live params while driving.

  python -m openpilot.sunnypilot.system.liveprobe.query summary [seconds=20]
  python -m openpilot.sunnypilot.system.liveprobe.query table [seconds=20] [step=1]
  python -m openpilot.sunnypilot.system.liveprobe.query mark [seconds=180] [note...]
  python -m openpilot.sunnypilot.system.liveprobe.query steer [seconds=60] [window=20]
  python -m openpilot.sunnypilot.system.liveprobe.query bookmarks [context_seconds=0]
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
  "LaneCenteringNarrowMode": (0.0, 1.0),
  "LaneCenteringJamMode": (0, 3),  # 0 off, 1 auto, 2 right, 3 left
  "LaneCenteringJamClearance": (-0.3, 0.5),  # m body to the line on the shift side, - = over it
  "LaneCenteringMinClearance": (0.0, 0.5),
  "LaneCenterOffset": (0.0, 0.3),
  "LaneCenterOffsetLeft": (-1.0, 0.4),  # negative = same as LaneCenterOffset
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
  "LatHighwayFrictionScale": (0.0, 1.5),  # 1.0 = unchanged; friction term above 60-72 km/h (ioniq6_starpilot.py)
  "ModelCrawlLatDeadband": (0.0, 0.1),
  "LatBendTrimGain": (0.0, 1.5),  # 0 = off; 1.0 = replay setting (bend_trim.py)
  "LatBendTrimDropout": (0.0, 1.0),  # 0 = off; 1 = trim through line dropouts (single line + 1.5 s hold)
  "LatCornerSpeedWarnAccel": (0.0, 4.0),  # 0 = off; 2.8 = replay setting (corner_speed_warning.py)
  "LatSplitPathHz": (0.0, 0.6),  # 0 = off; 0.35-0.6 = scene low-pass cutoff (split_path.py)
  "LatSplitPathLowHz": (0.0, 0.3),  # 0 = off; 0.15 = sim setting: the split leaves the lane band below this alone
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
  ("cam", "camera shift applied m (+ left)", 1, "{:+.2f}"),
  ("bt_w", "bend trim weight", 1, "{:.2f}"),
  ("bt_corr", "bend trim correction m/s2 (+ right)", 1, "{:+.2f}"),
  ("bt_off", "bend trim observer offset m (+ right)", 1, "{:+.2f}"),
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
         ("off", "off", 1, "{:+5.2f}"), ("off_target", "tgt", 1, "{:+5.2f}"), ("cam", "cam", 1, "{:+5.2f}"),
         ("bt_w", "btw", 1, "{:4.2f}"), ("bt_corr", "btc", 1, "{:+5.2f}"), ("des_curv", "dcur", 1e3, "{:+6.1f}"),
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


def fetch_steer(seconds: float, path: str = SOCKET_PATH) -> dict:
  s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
  s.settimeout(5.0)
  s.connect(path)
  s.sendall(f"steer {seconds}".encode())
  data = b""
  while chunk := s.recv(1 << 20):
    data += chunk
  s.close()
  return json.loads(data)


def fetch_bookmarks(path: str = SOCKET_PATH) -> dict:
  s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
  s.settimeout(3.0)
  s.connect(path)
  s.sendall(b"bookmarks")
  data = b""
  while chunk := s.recv(65536):
    data += chunk
  s.close()
  return json.loads(data)


def bookmarks(data: dict, rows: list[dict] | None = None, context: float = 0.0) -> str:
  """Bookmarks pressed this drive (probe uptime), newest last; with rows, a short table around each one."""
  bms = data.get("bookmarks") or []
  if not bms:
    return "no bookmarks since the probe started"
  now = data["now"]
  out = [f"{len(bms)} bookmark(s):"]
  for b in bms:
    ago = now - b["t"]
    stamp = datetime.fromtimestamp(b["utc"], UTC).strftime("%H:%M:%S")
    out.append(f"  {ago:5.0f} s ago ({stamp} UTC)")
    if rows and context > 0:
      w = [r for r in rows if b["t"] - context <= r["t"] <= b["t"] + context / 3]
      if w:
        t_lines = table(w).splitlines()
        out += ["    " + line for line in t_lines]
  return "\n".join(out)


STEER_MIN_SPEED = 15.0  # m/s
STEER_FS = 100.0


def _band(x: np.ndarray, lo: float, hi: float, fs: float = STEER_FS) -> np.ndarray:
  """Zero-phase FFT band-pass of a detrended series."""
  n = len(x)
  if n < 16:
    return np.zeros(n)
  t = np.arange(n)
  x = x - np.polyval(np.polyfit(t, x, 1), t)
  spec = np.fft.rfft(x)
  f = np.fft.rfftfreq(n, 1 / fs)
  spec[(f < lo) | (f >= hi)] = 0
  return np.fft.irfft(spec, n)


def _reversals(x: np.ndarray, ok: np.ndarray, hyst: float = 0.0) -> float:
  """Sign changes per second of x over the ok samples (with a +-hyst band that must be crossed)."""
  n, last, secs = 0, 0, ok.sum() / STEER_FS
  for v, k in zip(x, ok, strict=True):
    if not k:
      continue
    sgn = 1 if v > hyst else -1 if v < -hyst else 0
    if sgn and last and sgn != last:
      n += 1
    if sgn:
      last = sgn
  return n / secs if secs > 0 else float("nan")


def steer_stats(d: dict[str, np.ndarray]) -> dict:
  ok = (d["lat"] > 0.5) & (d["pressed"] < 0.5) & (d["v"] > STEER_MIN_SPEED)
  r: dict = {"seconds": ok.sum() / STEER_FS}
  if ok.sum() < 5 * STEER_FS:
    return r
  ang, out = d["ang"], d["out"]

  def b(x: np.ndarray, lo: float, hi: float) -> float:
    return float(np.std(_band(x, lo, hi)[ok]))

  a = _band(ang, 0.2, 3.0)
  spec = np.abs(np.fft.rfft(a)) ** 2
  f = np.fft.rfftfreq(len(a), 1 / STEER_FS)
  sel = (f >= 0.2) & (f < 3.0)
  r.update(
    kph=float(np.median(d["v"][ok]) * 3.6),
    ang_lo=b(ang, 0.2, 0.8), ang_hi=b(ang, 0.8, 4.0), ang_peak_hz=float(f[sel][np.argmax(spec[sel])]),
    out_mean=float(np.mean(out[ok])), out_lo=b(out, 0.3, 0.8), out_hi=b(out, 0.8, 4.0),
    out_rev=_reversals(out, ok), out_rev_h=_reversals(out, ok, 0.005), near_zero=float(np.mean(np.abs(out[ok]) < 0.01)),
    eps_mean=float(np.mean(d["eps_tq"][ok])), eps_lo=b(d["eps_tq"], 0.3, 0.8), eps_hi=b(d["eps_tq"], 0.8, 4.0),
    eps_rev_h=_reversals(_band(d["eps_tq"], 0.05, 4.0) + np.mean(d["eps_tq"][ok]), ok, 0.3),
    drv_lo=b(d["drv_tq"], 0.3, 0.8),
    p_mean=float(np.mean(d["p"][ok])), i_mean=float(np.mean(d["i"][ok])), f_mean=float(np.mean(d["f"][ok])),
    p_lo=b(d["p"], 0.3, 0.8), f_lo=b(d["f"], 0.3, 0.8), p_hi=b(d["p"], 0.8, 4.0), f_hi=b(d["f"], 0.8, 4.0),
  )
  return r


STEER_HELP = """reference, whole highway segments 70-145 km/h, hands off (routes 275/276/282): torque reversals median
  1.0-1.3/s (beyond +-0.005: 0.7-0.9), near zero 17-21%; worst segments 1.8-2.4/s with near zero 33-39%."""


def steer(data: dict, window: float = 20.0) -> str:
  rows = data.get("rows") or []
  if not rows:
    return "no steering data (probe not running or not onroad)"
  arr = np.asarray(rows, dtype=float)
  d = {k: arr[:, j] for j, k in enumerate(data["fields"])}
  t = d["t"] - d["t"][-1]
  hz = (len(t) - 1) / max(t[-1] - t[0], 1e-3)
  st = steer_stats(d)
  out = [f"last {-t[0]:.0f} s, {len(t)} rows ({hz:.0f} Hz), hands-off engaged above {STEER_MIN_SPEED * 3.6:.0f} km/h: {st['seconds']:.0f} s"]
  if "kph" not in st:
    out.append("  too little hands-off engaged time above the speed limit")
    return "\n".join(out)
  torque = (f"  TORQUE command: reversals {st['out_rev']:.1f}/s (beyond +-0.005: {st['out_rev_h']:.1f}/s), "
            + f"near zero (|out|<0.01) {st['near_zero']:.0%}, mean {st['out_mean']:+.3f}, "
            + f"swing 0.3-0.8 Hz {st['out_lo']:.4f} / 0.8-4 Hz {st['out_hi']:.4f}")
  terms = (f"  terms: p mean {st['p_mean']:+.3f} swing {st['p_lo']:.3f}/{st['p_hi']:.3f} | i mean {st['i_mean']:+.3f} | "
           + f"f mean {st['f_mean']:+.3f} swing {st['f_lo']:.3f}/{st['f_hi']:.3f} (slow/fast)")
  eps = (f"  EPS motor torque: mean {st['eps_mean']:+.2f}, swing 0.3-0.8 Hz {st['eps_lo']:.2f} / 0.8-4 Hz {st['eps_hi']:.2f}, "
         + f"reversals {st['eps_rev_h']:.1f}/s | column (driver) torque 0.3-0.8 Hz {st['drv_lo']:.1f}")
  out += [
    f"  speed {st['kph']:.0f} km/h",
    torque,
    terms,
    f"  WHEEL angle: 0.2-0.8 Hz {st['ang_lo']:.3f} deg, 0.8-4 Hz {st['ang_hi']:.3f} deg, peak {st['ang_peak_hz']:.2f} Hz",
    eps,
    "  " + STEER_HELP,
  ]
  if window > 0 and -t[0] > 1.5 * window:
    out.append(f"  per {window:.0f} s:  t   km/h  rev/s  near0  out_mean  out_lo  ang_lo  ang_hi  f_mean  i_mean")
    for t0 in np.arange(t[0], 0.0, window):
      m = (t >= t0) & (t < t0 + window)
      w = steer_stats({k: v[m] for k, v in d.items()})
      if "kph" in w:
        out.append(f"           {t0:+5.0f}  {w['kph']:4.0f}  {w['out_rev']:5.1f}  {w['near_zero']:5.0%}  {w['out_mean']:+.3f}  "
                   + f"{w['out_lo']:.4f}  {w['ang_lo']:.3f}  {w['ang_hi']:.3f}  {w['f_mean']:+.3f}  {w['i_mean']:+.3f}")
  return "\n".join(out)


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
  btw = _vals(rows, "bt_w")
  if btw.size:
    out.append(f"  bend trim active (weight >0.5): {np.mean(btw > 0.5):.0%} of the time")
  bm = [r for r in rows if r.get("bookmark")]
  if bm:
    rel = ", ".join(f"{r['t'] - rows[-1]['t']:+.0f} s" for r in bm)
    out.append(f"  bookmarks: {rel}")
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


def mark(rows: list[dict], note: str, steer_data: dict | None = None, bookmark_data: dict | None = None) -> str:
  os.makedirs(MARK_DIR, exist_ok=True)
  path = os.path.join(MARK_DIR, time.strftime("%Y%m%d-%H%M%S", time.gmtime()) + ".json")
  with open(path, "w") as f:
    json.dump({"note": note, "rows": rows, "steer": steer_data, "bookmarks": bookmark_data}, f)
  n = len((steer_data or {}).get("rows") or [])
  return f"saved {len(rows)} rows and {n} steering rows to {path}"


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
    sec = float(argv[1]) if len(argv) > 1 else 180.0
    try:
      sd = fetch_steer(sec)
    except (OSError, ValueError):
      sd = None
    try:
      bd = fetch_bookmarks()
    except (OSError, ValueError):
      bd = None
    return mark(fetch(sec), " ".join(argv[2:]), sd, bd)
  if cmd == "bookmarks":
    ctx = float(argv[1]) if len(argv) > 1 else 0.0
    return bookmarks(fetch_bookmarks(), fetch(180.0) if ctx > 0 else None, ctx)
  if cmd == "steer":
    return steer(fetch_steer(float(argv[1]) if len(argv) > 1 else 60.0), float(argv[2]) if len(argv) > 2 else 20.0)
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
