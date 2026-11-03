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
    "modelDataV2SP": NS(modelDataV2SP=NS(cameraOffset=cam)),
    "modelV2": NS(modelV2=NS(laneLines=[_line(-5.0), _line(-1.6 - cam), _line(1.2 - cam), _line(5.0)],
                             laneLineProbs=[0.1, 0.3, 0.95, 0.2], roadEdges=[_line(-3.0 - cam), _line(2.5 - cam)],
                             position=_line(0.1), action=NS(desiredCurvature=0.002))),
    "radarWallsSP": NS(radarWallsSP=NS(left=NS(valid=wall_valid, y=wall_l), right=NS(valid=False, y=0.0))),
    "lateralTuneStateSP": NS(lateralTuneStateSP=NS(laneCentreActive=True, laneCentrePosition=4, laneCentreStatus=11,
                                                   laneCentreOffset=0.0, laneCentreOffsetTarget=0.0, laneCentreLimitCause=8,
                                                   laneCentreSlackRight=0.1, laneCentreSlackLeft=0.4,
                                                   laneCentreWallRight=False, laneCentreWallLeft=True)),
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
  assert q.set_param(p, "SteerHeadroomCan", "900", log).startswith("refused")
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
