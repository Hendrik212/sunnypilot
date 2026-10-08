from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.common.realtime import DT_CTRL, DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib import dynamic_lane_centering as dlc
from openpilot.sunnypilot.selfdrive.controls.lib.dynamic_lane_centering import (
  DynamicLaneCentering, LanePosition, LanePositionClassifier, frame_lane_position)

_XS = np.linspace(0.0, 50.0, 52)
V = 30.0


def _path(y, y_std=0.1):
  return SimpleNamespace(x=_XS.copy(), y=np.full_like(_XS, float(y)), yStd=np.full_like(_XS, float(y_std)))


def _model(lane_left=True, lane_right=True, shoulder_right=False, lane_change=0, inner_prob=0.95, model_y=0.0):
  """Lane lines at -5.3/-1.75/1.75/5.3. A neighbour lane = confident outer line + far edge.
  No lane = outer prob ~0 and the edge close (shoulder on the right: edge ~3 m out)."""
  probs = [0.85 if lane_left else 0.01, inner_prob, inner_prob, 0.85 if lane_right else 0.01]
  edge_l = -7.5 if lane_left else -2.5
  edge_r = 7.5 if lane_right else (4.8 if shoulder_right else 2.5)
  return SimpleNamespace(
    laneLines=[_path(-5.3), _path(-1.75), _path(1.75), _path(5.3)],
    laneLineProbs=probs,
    laneLineStds=[0.1, 0.1, 0.1, 0.1],
    roadEdges=[_path(edge_l), _path(edge_r)],
    position=_path(model_y),
    meta=SimpleNamespace(laneChangeState=lane_change),
  )


@pytest.mark.parametrize("left,right,expected", [
  (True, False, LanePosition.RIGHT),
  (True, True, LanePosition.MIDDLE),
  (False, True, LanePosition.LEFT),
  (False, False, LanePosition.SINGLE),
])
def test_frame_classification(left, right, expected):
  assert frame_lane_position(_model(lane_left=left, lane_right=right)) == expected


def test_frame_classification_on_capnp_message():
  # capnp lists can't be sliced; the SimpleNamespace fixtures above would not catch that
  from openpilot.cereal import messaging
  msg = messaging.new_message('modelV2')
  m = msg.modelV2
  ref = _model(lane_left=True, lane_right=False)
  for attr in ('laneLines', 'roadEdges'):
    src = getattr(ref, attr)
    dst = m.init(attr, len(src))
    for i, p in enumerate(src):
      dst[i].x = p.x.tolist()
      dst[i].y = p.y.tolist()
  m.laneLineProbs = ref.laneLineProbs
  assert frame_lane_position(m) == LanePosition.RIGHT


def test_hard_shoulder_is_not_a_lane():
  # outer-right prob ~0 with the edge a shoulder's width out: still the right lane
  assert frame_lane_position(_model(lane_right=False, shoulder_right=True)) == LanePosition.RIGHT


def test_weak_inner_lines_are_ambiguous():
  assert frame_lane_position(_model(inner_prob=0.4)) is None


def _feed(c, model, seconds, v=V):
  for _ in range(int(round(seconds / DT_MDL))):
    c.update(model, v)
  return c.position


def test_dwell_from_unknown():
  c = LanePositionClassifier(dwell=3.0, switch_dist=400.0)
  assert _feed(c, _model(lane_right=False), 2.8) == LanePosition.UNKNOWN
  assert _feed(c, _model(lane_right=False), 0.3) == LanePosition.RIGHT


def test_switch_without_lane_change_needs_distance():
  c = LanePositionClassifier(dwell=3.0, switch_dist=400.0)
  _feed(c, _model(lane_right=False), 4.0)
  # an on-ramp lane appears on the right for 300 m at 30 m/s (10 s): stays RIGHT
  assert _feed(c, _model(lane_right=True), 10.0) == LanePosition.RIGHT
  assert _feed(c, _model(lane_right=False), 1.0) == LanePosition.RIGHT
  # a real added lane persists past 400 m
  assert _feed(c, _model(lane_right=True), 14.0) == LanePosition.MIDDLE


def test_lane_change_resets_and_reacquires_fast():
  c = LanePositionClassifier(dwell=3.0, switch_dist=400.0)
  _feed(c, _model(lane_right=False), 4.0)
  _feed(c, _model(lane_change=2), 2.0)
  assert c.position == LanePosition.UNKNOWN
  assert _feed(c, _model(lane_left=True, lane_right=True), 3.1) == LanePosition.MIDDLE


def test_ambiguous_frames_do_not_break_dwell():
  c = LanePositionClassifier(dwell=3.0, switch_dist=400.0)
  _feed(c, _model(lane_right=False), 2.0)
  _feed(c, _model(inner_prob=0.4), 5.0)
  assert _feed(c, _model(lane_right=False), 1.1) == LanePosition.RIGHT


class _P:
  def __init__(self, **kw):
    self.kw = kw

  def get(self, key, return_default=False):
    return self.kw.get(key)

  def get_bool(self, key):
    return bool(self.kw.get(key, False))


def _dyn(**kw):
  d = DynamicLaneCentering()
  d.get_params(_P(**{"LaneCentering": True, "LaneCenterOffset": 0.15, "LaneCenteringE2EAuthority": 0.0, **kw}))
  return d


def _run(d, model, seconds, lat_active=True):
  out = 0.0
  for i in range(int(round(seconds / DT_CTRL))):
    out = d.update(0.0, model, i % 5 == 0, True, V, lat_active, False, False)
  return out


@pytest.mark.parametrize("left,right,sign", [(True, False, 1.0), (False, False, 1.0), (True, True, 0.0), (False, True, -1.0)])
def test_offset_target_by_lane(left, right, sign):
  d = _dyn()
  _run(d, _model(lane_left=left, lane_right=right), 10.0)
  assert d.offset_target == pytest.approx(sign * 0.15)
  assert d.offset == pytest.approx(sign * 0.15)


def test_offset_is_rate_limited():
  d = _dyn()
  _run(d, _model(lane_right=False), 3.0 + 1.0)  # acquired after 3 s, then 1 s of ramp
  assert 0.0 < d.offset <= dlc.OFFSET_RAMP_RATE * 1.05 + 1e-9


def test_right_lane_offset_steers_right():
  # model path centred, right-lane offset -> positive (right) correction once past the deadband
  d = _dyn(LaneCenterOffset=0.3, LaneCenteringDeadband=0.0)
  out = _run(d, _model(lane_right=False), 15.0)
  assert out > 0.0 and d.active


def test_disabled_is_identity():
  d = DynamicLaneCentering()
  d.get_params(_P(LaneCentering=False))
  assert d.update(0.001, _model(lane_right=False), True, True, V, True, False, False) == 0.001


def test_params_defaults_and_clipping():
  d = DynamicLaneCentering()
  d.get_params(_P(LaneCentering=True, LaneCenterOffset=1.0, LaneCenteringGain=None))
  assert d.offset_mag == 0.3
  assert d.controller.gain == 0.30


def test_geometry_only_recomputed_on_new_model_frames(monkeypatch):
  calls = []
  orig = dlc.LaneCenteringController._raw_correction
  monkeypatch.setattr(dlc.LaneCenteringController, "_raw_correction",
                      staticmethod(lambda *a, **k: calls.append(1) or orig(*a, **k)))
  d = _dyn()
  _run(d, _model(lane_right=False), 1.0)  # 100 frames, a new model frame every 5th
  assert len(calls) == 20


def test_status():
  St = dlc.LaneCenteringStatus
  assert DynamicLaneCentering().status == St.OFF
  d = _dyn(LaneCenterOffset=0.3, LaneCenteringDeadband=0.0, LaneCenteringPauseOnSignal=True)
  _run(d, _model(lane_right=False), 1.0, lat_active=False)
  assert d.status == St.STANDBY
  _run(d, _model(inner_prob=0.4), 1.0)
  assert d.status == St.NO_LINES
  _run(d, _model(lane_right=True), 2.0)  # MIDDLE not yet adopted: offset 0, path on the centre
  assert d.status == St.CENTERED
  _run(d, _model(lane_right=False), 15.0)
  assert d.status == St.NUDGE_RIGHT
  _run(d, _model(lane_right=False, model_y=0.6), 5.0)  # path well right of the target
  assert d.status == St.NUDGE_LEFT
  d.update(0.0, _model(lane_right=False), True, True, V, True, True, False)  # blinker
  assert d.status == St.PAUSED
  d.update(0.0, _model(lane_right=False), True, True, V, True, False, True)  # driver override
  assert d.status == St.PAUSED
