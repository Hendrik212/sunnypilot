"""Traffic-jam / emergency-corridor mode (dynamic_lane_centering.JamShift)."""
import numpy as np
import pytest

from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.controls.lib import dynamic_lane_centering as dlc
from openpilot.sunnypilot.selfdrive.controls.lib.dynamic_lane_centering import JamMode, LanePosition, LimitCause
from openpilot.sunnypilot.selfdrive.controls.lib.lane_centering import LaneCenteringStatus as St
from openpilot.sunnypilot.selfdrive.controls.lib.tests.test_dynamic_lane_centering import _P, _model, _path

HALF = dlc.DEFAULT_CAR_WIDTH / 2


def _jam(**kw):
  d = dlc.DynamicLaneCentering()
  d.get_params(_P(**{"LaneCentering": True, "LaneCenterOffset": 0.15, "LaneCenteringOffsetMode": 1, **kw}))
  return d


def _drive(d, model, seconds, v, lat_active=True, blindspot=(False, False)):
  for i in range(int(round(seconds / DT_CTRL))):
    d.update(0.0, model, i % 5 == 0, True, v, lat_active, False, False, d.camera_offset, (None, None), blindspot)


def _into_jam(d, model, slow=2.0, seconds=15.0):
  _drive(d, model, 5.0, 30.0)  # Autobahn first: lane position settles, AUTO is armed
  _drive(d, model, seconds, slow)


def test_right_lane_moves_right_over_the_line():
  d = _jam()
  m = _model(lane_right=False)  # RIGHT lane, 3.5 m, road edge 2.5 m right of the centre
  _into_jam(d, m)
  assert d.position == LanePosition.RIGHT
  want = 1.75 - HALF - dlc.DEFAULT_JAM_CLEARANCE  # 0.10 m over the line
  assert d.offset_target == pytest.approx(want, abs=1e-6)
  assert d.offset == pytest.approx(want, abs=1e-3)
  assert d.camera_offset == pytest.approx(-want, abs=1e-3)  # beyond the 0.50 legacy cap
  assert d.status == St.JAM_RIGHT


def test_left_lane_moves_left():
  d = _jam()
  _into_jam(d, _model(lane_left=False))
  assert d.position == LanePosition.LEFT
  assert d.offset == pytest.approx(-(1.75 - HALF + 0.10), abs=1e-3)
  assert d.status == St.JAM_LEFT


def test_middle_lane_auto_stays_centred_manual_stops_at_the_line():
  d = _jam()
  m = _model()
  _into_jam(d, m)
  assert d.position == LanePosition.MIDDLE
  assert d.offset_target == pytest.approx(-dlc.DEFAULT_MIDDLE_BIAS)
  d.jam.mode = JamMode.RIGHT
  _drive(d, m, 15.0, 2.0)
  assert d.offset_target == pytest.approx(1.75 - HALF, abs=1e-6)  # body at the line, not over it


def test_never_closer_than_the_edge_margin():
  d = _jam()
  m = _model(lane_right=False)
  m.roadEdges[1] = _path(1.9)  # kerb 0.15 m beyond the line
  _into_jam(d, m)
  assert d.offset_target == pytest.approx(1.9 - HALF - dlc.JAM_EDGE_MARGIN, abs=1e-6)
  assert d.limit_cause == LimitCause.RIGHT_EDGE
  assert d.status == St.LIMITED


def test_no_confident_edge_no_crossing():
  d = _jam()
  m = _model(lane_right=False)
  m.roadEdgeStds = [0.3, 2.0]
  _into_jam(d, m)
  assert d.offset_target == pytest.approx(1.75 - HALF, abs=1e-6)


def test_clearance_is_live():
  d = _jam(LaneCenteringJamClearance=0.2)
  _into_jam(d, _model(lane_right=False))
  assert d.offset_target == pytest.approx(1.75 - HALF - 0.2, abs=1e-6)
  assert d.jam.clearance == 0.2
  assert _jam(LaneCenteringJamClearance=-5).jam.clearance == dlc.JAM_CLEARANCE_RANGE[0]


def test_auto_needs_recent_fast_driving():
  d = _jam()
  m = _model(lane_right=False)
  _drive(d, m, 20.0, 12.0)  # town at 43 km/h, then slow
  _drive(d, m, 10.0, 5.0)
  assert not d.jam.active
  assert d.offset_target == pytest.approx(0.15)  # the normal right-lane offset


def test_manual_works_without_fast_driving_and_resets_after_the_jam():
  d = _jam(LaneCenteringJamMode=2)
  m = _model(lane_right=False)
  _drive(d, m, 10.0, 2.0)
  assert d.jam_on and d.offset > 0.5
  _drive(d, m, 2.0, 15.0)
  assert d.jam.active  # off only after JAM_OFF_HOLD
  _drive(d, m, 2.0, 15.0)
  assert not d.jam.active and d.jam.reset_manual
  assert abs(d.camera_offset) > dlc.LEGACY_CAMERA_CAP  # ramps back, no step at the legacy cap
  _drive(d, m, 15.0, 15.0)
  assert d.offset == pytest.approx(0.15, abs=1e-3)


def test_blind_spot_holds_the_offset():
  d = _jam()
  m = _model(lane_right=False)
  _drive(d, m, 5.0, 30.0)
  _drive(d, m, 3.0, 2.0)
  held = d.offset
  _drive(d, m, 5.0, 2.0, blindspot=(False, True))
  assert d.offset == pytest.approx(held, abs=2e-3)
  assert d.limit_cause == LimitCause.BLINDSPOT_RIGHT
  _drive(d, m, 5.0, 2.0, blindspot=(True, False))  # a car on the left does not matter
  assert d.offset > held + 0.3


def test_not_engaged_ramps_back():
  d = _jam()
  m = _model(lane_right=False)
  _into_jam(d, m)
  _drive(d, m, 12.0, 2.0, lat_active=False)
  assert d.offset == pytest.approx(0.15, abs=1e-3)


def test_off_and_legacy_cap():
  d = _jam(LaneCenteringJamMode=0)
  _into_jam(d, _model(lane_right=False))
  assert not d.jam.active and d.offset == pytest.approx(0.15, abs=1e-3)
  assert d._camera_clip(1.2) == dlc.LEGACY_CAMERA_CAP


def test_rolling_plan_bounds_the_shift():
  d = _jam()
  m = _model(lane_right=False)
  _drive(d, m, 5.0, 30.0)
  # rolling at 6 m/s with a plan already 0.5 m right of the lane centre: the plan's line gap limits
  m.position = _path(0.5)
  _drive(d, m, 20.0, 6.0)
  assert d.limit_cause == LimitCause.RIGHT_LINE
  assert d.offset_target < 1.75 - HALF + 0.10 - 0.1
  assert np.isfinite(d.offset_target)
