import threading
import time
from types import SimpleNamespace as NS

import numpy as np
import pytest

from openpilot.sunnypilot.system.liveprobe import liveprobe as lp
from openpilot.sunnypilot.system.liveprobe import query as q

XS = np.linspace(0.0, 50.0, 33)


def _line(y):
  return NS(x=XS, y=np.full_like(XS, y))


def _msgs(cam=0.0, wall_l=-1.6, wall_valid=True):
  return {
    "carState": NS(carState=NS(vEgo=20.0, steeringPressed=False, steeringTorque=-87.0, leftBlinker=False, rightBlinker=False,
                               leftBlindspot=False, rightBlindspot=True)),
    "modelDataV2SP": NS(modelDataV2SP=NS(cameraOffset=cam, bendTrimCorrection=0.12, bendTrimWeight=0.8, bendTrimOffset=-0.25,
                                         splitPathReaction=0.0, splitPathWeight=0.0)),
    "modelV2": NS(modelV2=NS(laneLines=[_line(-5.0), _line(-1.6 - cam), _line(1.2 - cam), _line(5.0)],
                             laneLineProbs=[0.1, 0.3, 0.95, 0.2], roadEdges=[_line(-3.0 - cam), _line(2.5 - cam)],
                             position=_line(0.1), action=NS(desiredCurvature=0.002))),
    "radarWallsSP": NS(radarWallsSP=NS(left=NS(valid=wall_valid, y=wall_l), right=NS(valid=False, y=0.0))),
    "lateralTuneStateSP": NS(lateralTuneStateSP=NS(laneCentreActive=True, laneCentrePosition=4, laneCentreStatus=11,
                                                   laneCentreOffset=0.0, laneCentreOffsetTarget=0.0, laneCentreLimitCause=8,
                                                   laneCentreSlackRight=0.1, laneCentreSlackLeft=0.4,
                                                   laneCentreWallRight=False, laneCentreWallLeft=True,
                                                   laneCentreCameraOffset=0.0, laneCentreCorrection=0.0)),
    "carControl": NS(carControl=NS(latActive=True, actuators=NS(torque=1.2))),
  }


def test_sample_is_in_the_real_frame_with_body_gaps():
  r = lp.sample(_msgs(cam=-0.2), 0.94)
  # lines are shifted by -cam in the model frame; the probe adds the camera offset back
  assert r["line_l"] == pytest.approx(-1.6) and r["line_r"] == pytest.approx(1.2)
  assert r["lane_w"] == pytest.approx(2.8)
  assert r["gap_line_l"] == pytest.approx(0.66) and r["gap_line_r"] == pytest.approx(0.26)
  assert r["gap_wall_l"] == pytest.approx(0.66) and r["gap_wall_r"] is None
  assert r["gap_edge_l"] == pytest.approx(2.06) and r["gap_edge_r"] == pytest.approx(1.56)
  assert r["v_kph"] == pytest.approx(72.0) and r["cause"] == 8 and r["bs_r"] is True
  assert r["bt_w"] == pytest.approx(0.8) and r["bt_corr"] == pytest.approx(0.12) and r["bt_off"] == pytest.approx(-0.25)


def test_missing_services_leave_fields_out():
  r = lp.sample({}, 0.94)
  assert "lane_w" not in r and "gap_wall_l" not in r and r["cam"] == 0.0


def test_ring_buffer_keeps_only_the_window_and_answers_requests():
  p = lp.Probe(buffer_seconds=2.0, hz=5)
  for i in range(20):
    p.add({"t": float(i) * 0.2, "v_kph": i})
  assert len(p.rows) == 10
  assert [r["v_kph"] for r in p.last(0.5)] == [17, 18, 19]
  assert '"error"' in p.handle("bogus")


def test_socket_round_trip(tmp_path):
  p = lp.Probe()
  p.add({"t": time.monotonic(), "v_kph": 50.0})
  path = str(tmp_path / "probe.sock")
  end = threading.Event()
  th = threading.Thread(target=lp.serve, args=(p, end, path), daemon=True)
  th.start()
  for _ in range(50):
    try:
      rows = q.fetch(10.0, path)
      break
    except (FileNotFoundError, ConnectionRefusedError):
      time.sleep(0.02)
  end.set()
  assert rows[0]["v_kph"] == 50.0


def test_summary_and_table_read_like_the_analysis_tables():
  t0 = 1000.0
  rows = []
  for i in range(50):
    r = lp.sample(_msgs(), 0.94)
    r["t"] = t0 + i * 0.2
    rows.append(r)
  s = q.summary(rows)
  assert "body to left wall m: median 0.66" in s and "limit cause: wall no-shift L 100%" in s
  assert "lane position: left 100%" in s and "blind spot L/R: 0% / 100%" in s
  t = q.table(rows)
  assert t.splitlines()[0].split()[:3] == ["t", "kmh", "wallL"] and len(t.splitlines()) == 11
  assert q.summary([]).startswith("no data")


class _Params:
  def __init__(self, **kw):
    self.d = dict(kw)

  def get(self, key, return_default=False):
    return self.d.get(key)

  def put(self, key, value):
    self.d[key] = value


def test_set_is_whitelisted_ranged_typed_and_undo_walks_back(tmp_path):
  log = str(tmp_path / "changes.jsonl")
  p = _Params(LaneCenteringWallClearance=0.6, SteerDriverAllowanceLow=75)
  assert q.set_param(p, "LatAccelFactorCap", "3.0", log).startswith("refused")
  assert q.set_param(p, "SteerHeadroomCan", "1100", log).startswith("refused")
  assert q.set_param(p, "LaneCenteringWallClearance", "2.0", log).startswith("refused")
  assert q.set_param(p, "LaneCenteringWallClearance", "0.8", log) == "LaneCenteringWallClearance: 0.6 -> 0.8"
  assert q.set_param(p, "SteerDriverAllowanceLow", "150.4", log) == "SteerDriverAllowanceLow: 75 -> 150"
  assert isinstance(p.d["SteerDriverAllowanceLow"], int)
  assert q.set_param(p, "LaneCenteringWallClearance", "0.9", log).endswith("0.8 -> 0.9")
  assert q.undo(p, log).startswith("LaneCenteringWallClearance: 0.9 -> 0.8")
  assert q.undo(p, log).startswith("SteerDriverAllowanceLow: 150 -> 75")
  assert q.undo(p, log).startswith("LaneCenteringWallClearance: 0.8 -> 0.6")
  assert q.undo(p, log) == "nothing to undo"
  assert p.d == {"LaneCenteringWallClearance": 0.6, "SteerDriverAllowanceLow": 75}


def _steer_rows(seconds=60.0, out_bias=0.0, amp=0.02, f_hz=0.5):
  t = np.arange(0.0, seconds, 0.01)
  out = out_bias + amp * np.sin(2 * np.pi * f_hz * t)
  ang = 0.1 * np.sin(2 * np.pi * f_hz * t - 1.0)
  rows = []
  for k in range(len(t)):
    r = {"t": t[k], "v": 23.0, "ang": ang[k], "ang_rate": 0.0, "eps_tq": 10 * out[k], "drv_tq": 0.0, "pressed": 0.0, "lat": 1.0,
         "out": out[k], "p": 0.0, "i": 0.2, "f": -0.2, "des_la": 0.0, "act_la": 0.0}
    rows.append([r[f] for f in lp.STEER_FIELDS])
  return {"fields": list(lp.STEER_FIELDS), "rows": rows}


def test_steer_counts_torque_reversals_and_wheel_swing():
  d = _steer_rows()
  arr = np.asarray(d["rows"])
  st = q.steer_stats({k: arr[:, j] for j, k in enumerate(d["fields"])})
  assert st["out_rev"] == pytest.approx(1.0, abs=0.05)  # 0.5 Hz sine around zero: 2 sign changes per cycle
  assert st["ang_peak_hz"] == pytest.approx(0.5, abs=0.05)
  assert st["ang_lo"] == pytest.approx(0.1 / np.sqrt(2), rel=0.05)
  # the same swing on a torque bias never reverses
  d2 = _steer_rows(out_bias=0.05)
  arr2 = np.asarray(d2["rows"])
  assert q.steer_stats({k: arr2[:, j] for j, k in enumerate(d2["fields"])})["out_rev"] == 0.0
  text = q.steer(d)
  assert "TORQUE command: reversals 1.0/s" in text and "WHEEL angle" in text and "EPS motor torque" in text


def test_steer_buffer_answers_over_the_request_protocol():
  p = lp.Probe()
  for k in range(300):
    p.add_steer(tuple([k * 0.01] + [0.0] * (len(lp.STEER_FIELDS) - 1)))
  d = __import__("json").loads(p.handle("steer 1"))
  assert d["fields"] == list(lp.STEER_FIELDS) and 100 <= len(d["rows"]) <= 102
  assert q.steer({"fields": d["fields"], "rows": []}).startswith("no steering data")


def test_bookmarks_are_kept_once_and_listed_with_context():
  p = lp.Probe()
  t0 = 1000.0
  for i in range(100):
    r = lp.sample(_msgs(), 0.94)
    r["t"] = t0 + i * 0.2
    if i == 60:
      r["bookmark"] = "userBookmark,bookmarkButton"
    p.add(r)
  p.add_bookmark(t0 + 12.0, 1.7e9, "userBookmark,bookmarkButton")
  d = __import__("json").loads(p.handle("bookmarks"))
  assert len(d["bookmarks"]) == 1 and d["bookmarks"][0]["t"] == t0 + 12.0
  d["now"] = t0 + 20.0
  text = q.bookmarks(d, p.last(180.0), 5.0)
  assert "8 s ago" in text and "btw" in text
  assert "bookmarks: -8 s" in q.summary(p.last(180.0))
  assert "bend trim active (weight >0.5): 100%" in q.summary(p.last(180.0))
  assert q.bookmarks({"now": 0.0, "bookmarks": []}).startswith("no bookmarks")
