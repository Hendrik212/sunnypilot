"""Live probe: the last BUFFER_SECONDS of lane, wall and steering values, queryable while driving.

A voice request ("still too close to the left wall") reaches the analysis side 10-30 s after the
moment it is about, so sampling on demand is too late. This keeps a ring buffer of SAMPLE_HZ
rows in memory and answers over a unix socket (query.py: summary / table / mark / set / undo).

Load: core 4 is saturated when engaged (controlsd, card, selfdrived), so the probe pins itself
to the little cores at low priority and reads conflated sockets -- only the newest message of
each service is decoded, SAMPLE_HZ times a second. Runs onroad while the LiveProbe param is set;
selfdrived ignores it in processNotRunning, so a probe crash never reaches the driver.
"""
import json
import os
import socket
import threading
import time
from collections import deque
from datetime import UTC, datetime

import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog

SAMPLE_HZ = 5
BUFFER_SECONDS = 180
SOCKET_PATH = "/tmp/liveprobe.sock"
CORES = {0, 1, 2, 3}
NICE = 10
SAMPLE_X = 10.0  # m ahead for the plan
STALE_SECONDS = 1.0  # a service silent for longer drops out of the row

SERVICES = ("modelV2", "modelDataV2SP", "radarWallsSP", "lateralTuneStateSP", "carState", "controlsState",
            "carControl", "carOutput", "liveLocationKalman", "gpsLocationExternal", "selfdriveState")


def _at(line, x: float) -> float:
  xs, ys = np.asarray(line.x), np.asarray(line.y)
  return float(np.interp(x, xs, ys)) if xs.size else float("nan")


def _nan(v) -> float | None:
  return None if v is None or not np.isfinite(v) else round(float(v), 4)


def sample(msgs: dict, half_width: float) -> dict:
  """One row from the newest message of each service (missing services give None fields).
  Frame: y positive to the RIGHT, offsets + = right; body gaps are car body to the boundary."""
  row: dict = {"t": time.monotonic(), "utc": datetime.now(UTC).timestamp()}
  cs = msgs.get("carState")
  if cs is not None:
    c = cs.carState
    row.update(v_kph=c.vEgo * 3.6, pressed=c.steeringPressed, drv_tq=c.steeringTorque,
               blinker=-1 if c.leftBlinker else 1 if c.rightBlinker else 0, bs_l=c.leftBlindspot, bs_r=c.rightBlindspot)
  sp = msgs.get("modelDataV2SP")
  cam = sp.modelDataV2SP.cameraOffset if sp is not None else 0.0
  row["cam"] = cam
  mv = msgs.get("modelV2")
  if mv is not None:
    m = mv.modelV2
    if len(m.laneLines) >= 4 and len(m.laneLineProbs) >= 4:
      left, right = _at(m.laneLines[1], 0.0) + cam, _at(m.laneLines[2], 0.0) + cam
      row.update(line_l=left, line_r=right, lane_w=right - left, gap_line_l=-left - half_width, gap_line_r=right - half_width,
                 p_l=m.laneLineProbs[1], p_r=m.laneLineProbs[2], p_ll=m.laneLineProbs[0], p_rr=m.laneLineProbs[3])
    if len(m.roadEdges) >= 2:
      el, er = _at(m.roadEdges[0], 0.0) + cam, _at(m.roadEdges[1], 0.0) + cam
      row.update(gap_edge_l=-el - half_width, gap_edge_r=er - half_width)
    if len(m.position.x):
      row["plan10"] = _at(m.position, SAMPLE_X)
    row["model_curv"] = m.action.desiredCurvature
  rw = msgs.get("radarWallsSP")
  if rw is not None:
    w = rw.radarWallsSP
    row["wall_l"] = w.left.y if w.left.valid else None
    row["wall_r"] = w.right.y if w.right.valid else None
    row["gap_wall_l"] = -w.left.y - half_width if w.left.valid else None
    row["gap_wall_r"] = w.right.y - half_width if w.right.valid else None
  lt = msgs.get("lateralTuneStateSP")
  if lt is not None:
    s = lt.lateralTuneStateSP
    row.update(lc_active=s.laneCentreActive, lc_pos=s.laneCentrePosition, lc_status=s.laneCentreStatus,
               off=s.laneCentreOffset, off_target=s.laneCentreOffsetTarget, cause=s.laneCentreLimitCause,
               slack_r=s.laneCentreSlackRight, slack_l=s.laneCentreSlackLeft,
               lc_wall_r=s.laneCentreWallRight, lc_wall_l=s.laneCentreWallLeft)
  ct = msgs.get("controlsState")
  if ct is not None:
    row.update(des_curv=ct.controlsState.desiredCurvature, curv=ct.controlsState.curvature)
  cc = msgs.get("carControl")
  if cc is not None:
    row.update(lat=cc.carControl.latActive, tq_req=cc.carControl.actuators.torque)
  co = msgs.get("carOutput")
  if co is not None:
    row["tq_out"] = co.carOutput.actuatorsOutput.torque
  llk = msgs.get("liveLocationKalman")
  if llk is not None and row.get("v_kph"):
    yaw = llk.liveLocationKalman.angularVelocityCalibrated.value
    if len(yaw) == 3:
      row["act_curv"] = yaw[2] / max(row["v_kph"] / 3.6, 1.0)
  gps = msgs.get("gpsLocationExternal")
  if gps is not None and gps.gpsLocationExternal.hasFix:
    row.update(lat_deg=gps.gpsLocationExternal.latitude, lon_deg=gps.gpsLocationExternal.longitude)
  sd = msgs.get("selfdriveState")
  if sd is not None and sd.selfdriveState.alertText1:
    row["alert"] = sd.selfdriveState.alertText1
  for k, v in list(row.items()):
    if isinstance(v, float):
      row[k] = _nan(v)
  return row


class Probe:
  def __init__(self, buffer_seconds: float = BUFFER_SECONDS, hz: float = SAMPLE_HZ):
    self.rows: deque = deque(maxlen=int(buffer_seconds * hz))
    self.lock = threading.Lock()

  def add(self, row: dict) -> None:
    with self.lock:
      self.rows.append(row)

  def last(self, seconds: float) -> list[dict]:
    with self.lock:
      if not self.rows:
        return []
      t_end = self.rows[-1]["t"]
      return [r for r in self.rows if r["t"] >= t_end - seconds]

  def handle(self, request: str) -> str:
    parts = request.split()
    if not parts or parts[0] != "rows":
      return json.dumps({"error": f"unknown request {request!r}"})
    seconds = float(parts[1]) if len(parts) > 1 else 20.0
    return json.dumps(self.last(seconds))


def serve(probe: Probe, end: threading.Event, path: str = SOCKET_PATH) -> None:
  if os.path.exists(path):
    os.unlink(path)
  srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
  srv.bind(path)
  srv.listen(2)
  srv.settimeout(1.0)
  while not end.is_set():
    try:
      conn, _ = srv.accept()
    except TimeoutError:
      continue
    with conn:
      try:
        conn.settimeout(2.0)
        req = conn.recv(256).decode().strip()
        conn.sendall(probe.handle(req).encode())
      except OSError:
        pass
  srv.close()


def main() -> None:
  try:
    os.sched_setaffinity(0, CORES)
    os.nice(NICE)
  except (AttributeError, OSError):
    pass
  params = Params()
  try:
    half_width = float(params.get("LaneCenteringCarWidth", return_default=True)) / 2
  except (TypeError, ValueError):
    half_width = 0.94
  socks = {s: messaging.sub_sock(s, conflate=True) for s in SERVICES}
  probe, end = Probe(), threading.Event()
  threading.Thread(target=serve, args=(probe, end), daemon=True).start()
  latest: dict = {}
  try:
    while True:
      t = time.monotonic()
      for s, sock in socks.items():
        msg = messaging.recv_one_or_none(sock)
        if msg is not None:
          latest[s] = (msg, t)
      try:
        probe.add(sample({s: m for s, (m, ts) in latest.items() if t - ts < STALE_SECONDS}, half_width))
      except Exception:
        cloudlog.exception("liveprobe sample failed")
      time.sleep(max(0.0, 1.0 / SAMPLE_HZ - (time.monotonic() - t)))
  finally:
    end.set()


if __name__ == "__main__":
  main()
