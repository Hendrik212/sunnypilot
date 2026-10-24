from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.common.realtime import DT_CTRL, DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib import dynamic_lane_centering as dlc
from openpilot.sunnypilot.selfdrive.controls.lib.radar_walls import Wall
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
    roadEdgeStds=[0.3, 0.3],
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
  d.get_params(_P(**{"LaneCentering": True, "LaneCenterOffset": 0.15, "LaneCenteringE2EAuthority": 0.0,
                     "LaneCenteringOffsetMode": 0, **kw}))
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


def test_camera_mode_hands_the_offset_to_modeld():
  St = dlc.LaneCenteringStatus
  d = _dyn(LaneCenterOffset=0.3, LaneCenteringOffsetMode=1)
  out = _run(d, _model(lane_right=False, model_y=0.0), 15.0)
  assert out == 0.0 and d.correction == 0.0  # no curvature trim
  assert d.position == LanePosition.RIGHT and d.offset > 0.2
  assert d.camera_offset == -d.offset  # CameraOffset convention: + moves the model left
  assert d.status == St.HOLD_RIGHT
  _run(d, _model(lane_right=False, inner_prob=0.4), 0.2)
  assert d.status == St.NO_LINES and d.camera_offset < -0.2  # offset kept without lines
  _run(d, _model(lane_right=False), 1.0, lat_active=False)
  assert d.status == St.STANDBY and d.camera_offset < 0.0  # still applied while disengaged


def test_camera_mode_default_and_disable_ramps_out():
  d = DynamicLaneCentering()
  d.get_params(_P(LaneCentering=True, LaneCenterOffset=0.3))
  assert d.mode == dlc.OffsetMode.CAMERA
  _run(d, _model(lane_right=False), 15.0)
  start = d.camera_offset
  assert start < -0.2
  d.get_params(_P(LaneCentering=False, LaneCenterOffset=0.3))
  prev = start
  for _ in range(100):
    assert d.update(0.001, _model(lane_right=False), True, True, V, True, False, False) == 0.001
    assert 0.0 <= d.camera_offset - prev <= dlc.OFFSET_RAMP_RATE * DT_CTRL + 1e-12
    prev = d.camera_offset
  assert d.status == dlc.LaneCenteringStatus.OFF


def test_trim_mode_publishes_no_camera_offset():
  d = _dyn(LaneCenterOffset=0.3, LaneCenteringDeadband=0.0)
  _run(d, _model(lane_right=False), 15.0)
  assert d.camera_offset == 0.0 and d.correction > 0.0


def test_camera_mode_status_shifting_then_holding():
  St = dlc.LaneCenteringStatus
  d = _dyn(LaneCenterOffset=0.3, LaneCenteringOffsetMode=1)
  _run(d, _model(lane_right=False), 2.0)  # lane position not adopted yet
  assert d.status == St.CENTERED
  _run(d, _model(lane_right=False), 2.0)  # adopted after 3 s dwell, ramping at 5 cm/s
  assert d.status == St.SHIFT_RIGHT and 0.0 < d.offset < 0.3
  _run(d, _model(lane_right=False), 8.0)
  assert d.status == St.HOLD_RIGHT
  _run(d, _model(lane_right=True, lane_left=False), 3.5)  # LEFT needs 400 m before switching
  _run(d, _model(lane_right=True, lane_left=False), 12.0)
  assert d.position == LanePosition.LEFT and d.status == St.SHIFT_LEFT


def _cam_run(d, model, seconds, lat_active=True):
  """camera mode with an ideal modeld: the applied camera offset is what controlsd asked for"""
  for i in range(int(round(seconds / DT_CTRL))):
    d.update(0.0, model, i % 5 == 0, True, V, lat_active, False, False, d.camera_offset)


def test_plan_slack_real_frame():
  m = _model(lane_right=False)
  # lines at +-1.75 in the model frame, half width 0.94, margin 0.25; camera offset -0.25 (car right)
  sl = dlc.plan_slack(m, V, -0.25, 0.94, 0.25)
  assert sl.right == pytest.approx(1.75 - 0.25 - 0.94 - 0.25) and sl.right_cause == dlc.LimitCause.RIGHT_LINE
  assert sl.left == pytest.approx(1.75 + 0.25 - 0.94 - 0.25) and sl.left_cause == dlc.LimitCause.LEFT_LINE
  m.laneLineProbs = [0.0, 0.2, 0.2, 0.0]
  m.roadEdgeStds = [2.0, 2.0]
  sl = dlc.plan_slack(m, V, 0.0, 0.94, 0.25)
  assert sl.right is None and sl.left is None


def test_plan_slack_radar_wall_is_real_frame_and_flags_near():
  m = _model(lane_left=False)
  wall = Wall(y=-1.7, slope=0.0, curvature=0.0, residual=0.02, points=12)
  # camera +0.25: car shifted left, left line now 1.5 m away (slack 0.31) -- the wall is tighter
  sl = dlc.plan_slack(m, V, 0.25, 0.94, 0.25, (wall, None), 0.6)
  assert sl.left == pytest.approx(1.7 - 0.94 - 0.6) and sl.left_cause == dlc.LimitCause.LEFT_WALL
  assert sl.wall_left and sl.wall_near_left and not sl.wall_right


def test_guard_cuts_the_offset_when_the_plan_nears_the_line():
  St = dlc.LaneCenteringStatus
  d = _dyn(LaneCenterOffset=0.3, LaneCenteringOffsetMode=1)
  _cam_run(d, _model(lane_right=False), 15.0)
  assert d.offset == pytest.approx(0.3) and not d.limited
  # the plan drifts 0.5 m right (bend): body would be 1.75 - 0.3 - 0.5 - 0.94 = 0.01 m from the line
  _cam_run(d, _model(lane_right=False, model_y=0.5), 0.5)
  assert d.limited and d.status == St.LIMITED and d.offset < 0.3 - 0.05
  _cam_run(d, _model(lane_right=False, model_y=0.5), 3.0)
  assert d.offset < 0.1  # cut fast
  # clear again: regrow only after the hold, then at the normal ramp rate
  _cam_run(d, _model(lane_right=False), 1.5)
  low = d.offset
  _cam_run(d, _model(lane_right=False), 1.0)
  assert d.offset - low <= dlc.OFFSET_RAMP_RATE * 1.0 + 1e-6
  _cam_run(d, _model(lane_right=False), 15.0)
  assert d.offset == pytest.approx(0.3) and not d.limited


def test_guard_keeps_away_from_a_line_the_model_rides():
  d = _dyn(LaneCenterOffset=0.0, LaneCenteringOffsetMode=1)
  _cam_run(d, _model(lane_right=False, model_y=0.7), 10.0)  # 1.75 - 0.7 - 0.94 = 0.11 < 0.25
  assert d.offset == pytest.approx(-0.14, abs=0.01) and d.camera_offset == pytest.approx(0.14, abs=0.01)


def test_guard_opens_on_lane_change():
  d = _dyn(LaneCenterOffset=0.3, LaneCenteringOffsetMode=1)
  _cam_run(d, _model(lane_right=False, model_y=0.6), 5.0)
  assert d.guard.hi < 0.3
  _cam_run(d, _model(lane_right=False, lane_change=2), 0.1)
  assert d.guard.hi is None


def _pinched(g, right, left, single=False, blindspot=(False, False), n=50, **kw):
  g.single, g.blindspot = single, blindspot
  for _ in range(n):
    g.set_slack(dlc.Slack(right=right, left=left, right_cause=dlc.LimitCause.RIGHT_LINE, left_cause=dlc.LimitCause.LEFT_LINE, **kw))
    g.update(0.0)
  return g.apply(0.3)


def test_guard_window_clips_the_nominal_offset():
  g = dlc.ClearanceGuard()
  assert _pinched(g, 0.12, 0.5) == (pytest.approx(0.12), dlc.LimitCause.RIGHT_LINE)
  assert _pinched(dlc.ClearanceGuard(), 0.5, 0.5) == (pytest.approx(0.3), dlc.LimitCause.NONE)


def test_narrow_single_lane_keeps_the_centre_line_clear():
  # 10 cm short on both sides of a country road: the left (oncoming) side gets its margin, the
  # right line yields; the old balance put the deficit on both sides
  assert _pinched(dlc.ClearanceGuard(), -0.1, -0.1, single=True) == (pytest.approx(0.1), dlc.LimitCause.NARROW)
  # more short than the right line may give: the rest is split
  out, cause = _pinched(dlc.ClearanceGuard(), -0.3, -0.2, single=True)
  assert cause == dlc.LimitCause.NARROW and out == pytest.approx(-0.05 + 0.5 * (0.2 + 0.05))


def test_narrow_between_lanes_splits_with_a_slight_left_margin():
  assert _pinched(dlc.ClearanceGuard(), -0.15, -0.15) == (pytest.approx(dlc.NARROW_BIAS / 2), dlc.LimitCause.NARROW)


def test_narrow_with_a_near_wall_yields_the_far_line_unless_a_car_is_there():
  kw = {'wall_left': True, 'wall_near_left': True, 'wall_slack_left': -0.2}
  out, _ = _pinched(dlc.ClearanceGuard(), -0.2, -0.2, **kw)
  assert out == pytest.approx(0.2)  # wall margin kept, the right line gives 0.2 of its 0.45
  out, _ = _pinched(dlc.ClearanceGuard(), -0.2, -0.2, blindspot=(True, False), **kw)
  assert out == pytest.approx(0.0)  # car alongside on the right: split, no give
  # 0.5 m short both sides: the right line gives its 0.45, the rest is split
  out, _ = _pinched(dlc.ClearanceGuard(), -0.5, -0.5, **kw)
  assert out == pytest.approx(-0.05 + 0.5 * 0.55)
  out, _ = _pinched(dlc.ClearanceGuard(), 0.4, -1.0, **kw)
  assert out == pytest.approx(dlc.WALL_CAP)  # capped


def test_bounds_tighten_at_once_and_relax_slowly():
  g = dlc.ClearanceGuard()
  g.set_slack(dlc.Slack(right=0.5, left=0.5))
  g.set_slack(dlc.Slack(right=-0.1, left=0.5))
  assert g.hi == pytest.approx(-0.1)
  for _ in range(int(dlc.BOUND_RELAX_HOLD / DT_MDL) - 1):
    g.set_slack(dlc.Slack(right=0.5, left=0.5))
  assert g.hi == pytest.approx(-0.1)  # held after the pinch
  for _ in range(int(dlc.BOUND_RELAX_TAU / DT_MDL) + 1):
    g.set_slack(dlc.Slack(right=0.5, left=0.5))
  assert g.hi == pytest.approx(0.5 - 0.6 / np.e, abs=0.02)
  # a side that stops showing a boundary keeps its bound for BOUND_HOLD, then opens
  for _ in range(int(dlc.BOUND_HOLD / DT_MDL) - 1):
    g.set_slack(dlc.Slack(right=None, left=0.5))
  assert g.hi is not None
  g.set_slack(dlc.Slack(right=None, left=0.5))
  assert g.hi is None


def _narrow_model(cam, follow, wiggle=0.0, left=-1.17, right=1.28):
  """2.45 m lane, the model rides 5.5 cm left of its centre (real y 0 at camera offset 0), plus
  `wiggle` m of bend-to-bend drift. modeld shifts the lines by -cam; the plan follows `follow`
  of the shift (the model resists)."""
  m = _model(lane_left=False, lane_right=False)
  m.laneLines = [_path(-5.0 - cam), _path(left - cam), _path(right - cam), _path(5.0 - cam)]
  m.roadEdges = [_path(-4.0 - cam), _path(4.0 - cam)]
  m.position = _path(wiggle - follow * cam)
  return m


def _narrow_run(follow, seconds=40.0, guard=None):
  d = _dyn(LaneCenterOffset=0.25, LaneCenteringOffsetMode=1, LaneCenteringDwell=0.0)
  cam, track = 0.0, []
  for i in range(int(round(seconds / DT_CTRL))):
    upd = i % 5 == 0
    if upd:
      cam = max(min(d.camera_offset, cam + 0.1 * DT_MDL), cam - 0.1 * DT_MDL)  # modeld rate limit
    wiggle = 0.12 * np.sin(2 * np.pi * i * DT_CTRL / 8.0)  # left and right bends, 8 s apart
    d.update(0.0, _narrow_model(cam, follow, wiggle), upd, True, V, True, False, False, cam)
    track.append(wiggle - follow * cam)
  return d, np.array(track)


@pytest.mark.parametrize("follow", [1.0, 0.5])
def test_narrow_lane_with_bends_keeps_the_centre_line_side_clear(follow):
  # 2.45 m lane, bends: the old latched limits wound down and stuck in NARROW, and the balance
  # centred the car. Now it rides as far right as the right margin (0.25) allows, the rest
  # of the lane goes to the left (oncoming) side
  d, track = _narrow_run(follow)
  last = track[-int(16.0 / DT_CTRL):]  # two bend cycles
  left_clear, right_clear = last + 1.17 - 0.94, 1.28 - last - 0.94
  assert np.mean(right_clear) == pytest.approx(0.25, abs=0.03)
  assert np.mean(left_clear) > np.mean(right_clear)
  assert np.min(left_clear) > 0.17
  assert np.ptp(d.offset_target - d.offset) < 0.15


def _cam_run_walls(d, model, seconds, walls):
  for i in range(int(round(seconds / DT_CTRL))):
    d.update(0.0, model, i % 5 == 0, True, V, True, False, False, d.camera_offset, walls)


def test_no_shift_toward_a_near_wall():
  d = _dyn(LaneCenterOffset=0.25, LaneCenteringOffsetMode=1)
  wall = Wall(y=-2.6, slope=0.0, curvature=0.0, residual=0.02, points=12)  # 1.66 m from the body: not near yet
  _cam_run_walls(d, _model(lane_left=False), 15.0, (wall, None))  # LEFT lane -> offset left
  assert d.offset == pytest.approx(-0.25)
  wall = Wall(y=-1.85, slope=0.0, curvature=0.0, residual=0.02, points=12)  # 0.91 m from the body, 0.35 m beyond the line
  _cam_run_walls(d, _model(lane_left=False), 0.5, (wall, None))
  # frozen where it was: no further toward the wall, but no step back to the centre either
  _cam_run_walls(d, _model(lane_left=False), 3.0, (wall, None))
  assert d.offset == pytest.approx(-0.25, abs=1e-6)


def test_near_wall_freezes_and_a_flicker_does_not_step_the_offset():
  St = dlc.LaneCenteringStatus
  d = _dyn(LaneCenterOffset=0.25, LaneCenteringOffsetMode=1, LaneCenteringSwitchDist=0.0)
  far = Wall(y=-2.6, slope=0.0, curvature=0.0, residual=0.02, points=12)
  near = Wall(y=-2.0, slope=0.0, curvature=0.0, residual=0.02, points=12)  # 0.25 m beyond the left line
  _cam_run_walls(d, _model(), 15.0, (far, None))  # middle lane: offset 0
  assert d.offset == pytest.approx(0.0)
  # a near left wall appears while the lane position asks for a left offset: frozen at 0
  _cam_run_walls(d, _model(lane_left=False), 5.0, (near, None))
  assert d.offset == pytest.approx(0.0, abs=1e-6)
  assert d.status == St.LIMITED and d.limit_cause == dlc.LimitCause.WALL_NO_SHIFT_LEFT
  # route 23b: the wall flickers in and out every second -- the offset must not move
  offs = []
  for k in range(10):
    _cam_run_walls(d, _model(lane_left=False), 1.0, (near if k % 2 else None, None))
    offs.append(d.offset)
  assert np.ptp(offs) < 1e-6
  # gone for longer than WALL_HOLD: released at the normal ramp rate
  _cam_run_walls(d, _model(lane_left=False), dlc.WALL_HOLD + 1.0, (None, None))
  assert -dlc.OFFSET_RAMP_RATE * 1.1 - 1e-6 <= d.offset < -0.01


def test_narrow_moves_fast_only_when_a_side_is_inside_its_margin():
  g = dlc.ClearanceGuard()
  g.set_slack(dlc.Slack(right=0.05, left=-0.04))
  assert not g.urgent(0.1) and not g.urgent(-0.1)  # slightly inside: normal rate, no hunting
  g.set_slack(dlc.Slack(right=0.05, left=-0.1))
  assert g.urgent(0.1) and not g.urgent(-0.1)


def test_a_guardrail_beyond_the_lane_line_is_not_near():
  m = _model(lane_left=False)
  # body 0.81 m from the left line; a wall 1.2 m from the body sits 0.39 m beyond it: near
  near = Wall(y=-2.14, slope=0.0, curvature=0.0, residual=0.02, points=12)
  assert dlc.plan_slack(m, V, 0.0, 0.94, 0.25, (near, None), 0.6).wall_near_left
  # 0.6 m beyond the line (a guardrail behind the verge): only a boundary with its margin
  rail = Wall(y=-2.35, slope=0.0, curvature=0.0, residual=0.02, points=12)
  sl = dlc.plan_slack(m, V, 0.0, 0.94, 0.25, (rail, None), 0.6)
  assert sl.wall_left and not sl.wall_near_left
  # no confident line on that side: the wall is the boundary
  m.laneLineProbs = [0.01, 0.2, 0.95, 0.85]
  assert dlc.plan_slack(m, V, 0.0, 0.94, 0.25, (rail, None), 0.6).wall_near_left


@pytest.mark.parametrize("follow", [1.0, 0.5])
def test_wall_margin_keeps_distance(follow):
  d = _dyn(LaneCenterOffset=0.0, LaneCenteringOffsetMode=1, LaneCenteringDwell=0.0)
  wall = Wall(y=-1.3, slope=0.0, curvature=0.0, residual=0.02, points=12)  # body 0.36 m off, margin 0.6
  cam = 0.0
  for i in range(int(round(15.0 / DT_CTRL))):
    upd = i % 5 == 0
    if upd:
      cam = max(min(d.camera_offset, cam + 0.1 * DT_MDL), cam - 0.1 * DT_MDL)
    d.update(0.0, _narrow_model(cam, follow, left=-1.75, right=1.75), upd, True, V, True, False, False, cam, (wall, None))
  track = -follow * cam
  assert track + 1.3 - 0.94 == pytest.approx(0.6, abs=0.03)  # pushed right until the wall margin holds
  assert d.limit_cause == dlc.LimitCause.LEFT_WALL
