"""
Dynamic lane centering: StarPilot's LaneCenteringController (lane_centering.py) with the
lane offset chosen by which lane the car is in.

  RIGHT / SINGLE lane -> +offset (right of the lane centre)
  MIDDLE lane         ->  0      (dead centre)
  LEFT lane           -> -offset (left of the lane centre)
  UNKNOWN             ->  0

Lane position comes from the model's OUTER lane lines and road edges, per side:
  lane present: outer line prob >= 0.5, outer lane 2.8-4.6 m wide, road edge beyond it
  no lane:      outer line prob < 0.3, or road edge within 2.8 m of our line
Measured on routes 00000209/210/22a (2026-10-06): on the Autobahn the hard shoulder does
NOT show up as a lane (outer prob ~0, edge 2-4 m out), a real neighbour lane does (prob
>0.5, gap ~3.5 m, edge 5-8 m out). SINGLE (no lane either side: unmarked country road,
centre line not seen, ramp) gets the RIGHT offset, so country-road RIGHT<->SINGLE flicker
does not move the car.

Dwell: a class is adopted after `dwell` s of consistent frames when the position is unknown
(start, after every lane change), but a change between two known classes ALSO needs
`switch_dist` m of consistent evidence -- Autobahn on-/off-ramp lanes put a full lane on the
right for ~250-350 m and must not pull the car to the centre (3 s-only dwell: 36 spurious
switches/h on 209; with 400 m: 2.3/h). The applied offset is rate-limited, never a step.

Offset mode (LaneCenteringOffsetMode, live):
  1 camera (default): the offset is handed to modeld as a camera offset
    (lateralTuneStateSP.laneCentreCameraOffset), so the model itself drives the offset line;
    no curvature trim. Routes 232-235 (2026-10-13/15): with the trim the model fought back
    like a spring (0.2-0.5 m/s^2 per m), reached only half of a 0.30 m offset, and as soon as
    the trim exceeded ~0.015 m/s^2 the highway weave went from 0% to 18-34% of 20 s windows
    (middle lane, offset 0: no extra weave).
  0 trim: StarPilot's curvature trim toward the offset line (LaneCenteringController).
Both modelds apply it (selfdrive/modeld and sunnypilot/modeld_v2).

Clearance guard (camera mode): with the camera offset applied the model's lines are in the
shifted frame, real = model + applied camera offset (modelDataV2SP.cameraOffset); the car's
own track is the model's planned path. On every model frame the guard takes the smallest
body-to-line (and body-to-road-edge minus EDGE_EXTRA) distance along the plan 0.5-1.5 s
ahead -- the plan reacts at once to a camera shift, the car's current position would lag
and wind the limit up. Below LaneCenteringMinClearance it lowers that side's offset limit
at GUARD_RELEASE_RATE (down to KEEP_AWAY past 0, pushing away from a line the model rides on
its own); the limit regrows at OFFSET_RAMP_RATE after GUARD_HOLD s with GUARD_HYST of spare
clearance. Routes 237/238 (2026-10-16): offset -0.25 on country right bends put the body
within 10 cm of the right line in 45% of frames (12% without offset), p5 -0.08 m.
Radar walls (radar_walls.py, radarWallsSP) are a third boundary with their own margin
(LaneCenteringWallClearance), and the offset never moves toward a wall within WALL_NEAR of
the body: route 238 seg 15 shifted 0.25 m toward a construction wall the model's road edge
put 0.3-0.6 m too far out, leaving the body 0.55-0.65 m from the concrete.
Narrow lanes (both limits pinched): the offset goes to the balance point -- equal planned
slack both sides plus NARROW_BIAS more room on the left (oncoming) side -- low-passed over
NARROW_TAU and capped at NARROW_MAX. Splitting the two limits instead gave exactly 0 once both
had wound down to KEEP_AWAY: routes 239/23a (2026-10-16), 2.4-2.6 m lanes, the guard sat in
NARROW with offset 0 while the body was within 10 cm of the centre line 14% of the time, every
time with 15+ cm more room on the right (the model rides ~8 cm left, ~12 cm in left bends).
The balance moves at OFFSET_RAMP_RATE unless the side it moves away from is already inside its
margin; NARROW_TAU 3 s. Route 23b (2026-10-17) construction zone: with 1 s and the fast guard
rate the balance swung -0.16 -> +0.22 m within seconds as a flickering left wall (0.60 m
margin vs 0.25 for a line) swapped in and out of the left slack.
A near wall FREEZES the offset where it was when the wall appeared (moving away stays allowed)
for WALL_HOLD s after it was last seen. Forcing the target to 0 instead stepped it 0.25 <-> 0
every 1-2 s along a flickering guardrail (route 23b, 36:56-37:41).

Params (all live, PARAMS_UPDATE_PERIOD): LaneCentering, LaneCenteringOffsetMode, LaneCenterOffset, LaneCenteringGain,
LaneCenteringPauseOnSignal, LaneCenteringSmoothTau, LaneCenteringDeadband, LaneCenteringDwell,
LaneCenteringSwitchDist, LaneCenteringE2EAuthority,
LaneCenteringReleaseHold, LaneCenteringReleaseTau, LaneCenteringReleaseJerk,
LaneCenteringMinClearance, LaneCenteringCarWidth, LaneCenteringWallClearance. Frame: model y positive to the RIGHT.
"""
from dataclasses import dataclass
from enum import IntEnum

import numpy as np

from openpilot.cereal import log
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL, DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib.lane_centering import LaneCenteringController, LaneCenteringStatus

LaneChangeState = log.LaneChangeState

SAMPLE_X = 10.0  # m ahead where lane lines / road edges are sampled
INNER_MIN_PROB = 0.5
OUTER_LANE_PROB = 0.5
OUTER_NONE_PROB = 0.3
OUTER_GAP_MIN, OUTER_GAP_MAX = 2.8, 4.6
EDGE_BEYOND_OUTER = 0.3  # road edge must sit this far beyond the outer line
EDGE_NO_LANE = 2.8  # road edge closer than this to our line = no lane on that side
OFFSET_RAMP_RATE = 0.05  # m/s
# camera-mode clearance guard
DEFAULT_MIN_CLEARANCE = 0.25  # m, car body to lane-line centre (the paint is ~0.15 m wide)
DEFAULT_CAR_WIDTH = 1.88  # m, Ioniq 6 without mirrors
EDGE_EXTRA = 0.15  # m, extra margin to a road edge
DEFAULT_WALL_CLEARANCE = 0.60  # m, car body to a radar wall (radar_walls.py)
WALL_NEAR = 1.5  # m, body to a radar wall: no offset toward it
GUARD_HORIZON = (0.5, 0.75, 1.0, 1.25, 1.5)  # s along the planned path
GUARD_RELEASE_RATE = 0.2  # m/s
GUARD_HYST = 0.10  # m
GUARD_HOLD = 2.0  # s
KEEP_AWAY = 0.15  # m the guard may push past 0, away from a line
GUARD_OPEN = 0.35  # m, a limit at or beyond this is inactive (offset max is 0.30)
NARROW_BIAS = 0.05  # m more clearance on the left (oncoming) side than on the right in a narrow lane
NARROW_TAU = 3.0  # s, low-pass on the narrow-lane balance point (it feeds back through modeld)
WALL_HOLD = 3.0  # s a near wall keeps the offset frozen after it was last seen
NARROW_MAX = 0.30  # m; the balance watches both lines, and the plan only follows ~half a camera shift
LINE_MIN_PROB = 0.5
EDGE_MAX_STD = 0.5
# camera-mode status readout only
CAMERA_STATUS_MIN_PROB = 0.5
CAMERA_STATUS_SETTLED = 0.01  # m between applied and target offset
CAMERA_STATUS_CENTRED = 0.02  # m

DEFAULT_OFFSET = 0.15
DEFAULT_DWELL = 3.0
DEFAULT_SWITCH_DIST = 400.0


class OffsetMode(IntEnum):
  TRIM = 0
  CAMERA = 1


class LanePosition(IntEnum):
  UNKNOWN = 0
  SINGLE = 1
  RIGHT = 2
  MIDDLE = 3
  LEFT = 4


OFFSET_SIGN = {LanePosition.RIGHT: 1.0, LanePosition.SINGLE: 1.0, LanePosition.LEFT: -1.0}


def _side(p_outer: float, gap: float, edge_dist: float) -> int:
  """+1 lane present, -1 no lane, 0 ambiguous."""
  if p_outer >= OUTER_LANE_PROB and OUTER_GAP_MIN <= gap <= OUTER_GAP_MAX and edge_dist >= gap + EDGE_BEYOND_OUTER:
    return 1
  if p_outer < OUTER_NONE_PROB or edge_dist < EDGE_NO_LANE:
    return -1
  return 0


def _sample(line) -> float:
  return float(np.interp(SAMPLE_X, np.asarray(line.x, dtype=float), np.asarray(line.y, dtype=float)))


def frame_lane_position(model_v2) -> LanePosition | None:
  """Per-frame lane position, None when the frame is ambiguous or unusable."""
  try:
    lines, edges = model_v2.laneLines, model_v2.roadEdges
    probs = np.asarray(model_v2.laneLineProbs, dtype=float)
    if len(lines) < 4 or len(edges) < 2 or probs.size < 4:
      return None
    if not np.isfinite(probs).all() or probs[1] < INNER_MIN_PROB or probs[2] < INNER_MIN_PROB:
      return None
    # index, don't slice: capnp lists do not support slicing
    y = [_sample(lines[i]) for i in range(4)]
    e = [_sample(edges[i]) for i in range(2)]
    if not np.isfinite(y + e).all():
      return None
  except (AttributeError, IndexError, TypeError, ValueError):
    return None

  left = _side(probs[0], y[1] - y[0], y[1] - e[0])
  right = _side(probs[3], y[3] - y[2], e[1] - y[2])
  if left == 0 or right == 0:
    return None
  return {(1, 1): LanePosition.MIDDLE, (1, -1): LanePosition.RIGHT,
          (-1, 1): LanePosition.LEFT, (-1, -1): LanePosition.SINGLE}[(left, right)]


class LanePositionClassifier:
  """Debounced lane position. Call update() once per NEW modelV2 frame."""

  def __init__(self, dwell: float = DEFAULT_DWELL, switch_dist: float = DEFAULT_SWITCH_DIST):
    self.dwell = dwell
    self.switch_dist = switch_dist
    self.reset()

  def reset(self) -> None:
    self.position = LanePosition.UNKNOWN
    self._candidate: LanePosition | None = None
    self._cand_time = 0.0
    self._cand_dist = 0.0

  def update(self, model_v2, v_ego: float, dt: float = DT_MDL) -> LanePosition:
    try:
      lane_change = model_v2.meta.laneChangeState in (LaneChangeState.laneChangeStarting,
                                                      LaneChangeState.laneChangeFinishing)
    except AttributeError:
      lane_change = False
    if lane_change:
      self.reset()
      return self.position

    frame = frame_lane_position(model_v2)
    if frame is None:  # ambiguous frames neither confirm nor break the candidate
      return self.position
    if frame != self._candidate:
      self._candidate, self._cand_time, self._cand_dist = frame, 0.0, 0.0
    self._cand_time += dt
    self._cand_dist += dt * max(float(v_ego), 0.0)

    if self._candidate != self.position and self._cand_time >= self.dwell:
      if self.position == LanePosition.UNKNOWN or self._cand_dist >= self.switch_dist:
        self.position = self._candidate
    return self.position


def _get_float(params: Params, key: str, default: float, lo: float, hi: float) -> float:
  try:
    v = params.get(key, return_default=True)
    v = float(v) if v is not None else default
  except (TypeError, ValueError):
    v = default
  return float(np.clip(v, lo, hi)) if np.isfinite(v) else default


def _interp(x, line) -> np.ndarray:
  return np.interp(x, np.asarray(line.x, dtype=float), np.asarray(line.y, dtype=float))


class LimitCause(IntEnum):
  NONE = 0
  RIGHT_LINE = 1
  LEFT_LINE = 2
  RIGHT_EDGE = 3
  LEFT_EDGE = 4
  RIGHT_WALL = 5
  LEFT_WALL = 6
  WALL_NO_SHIFT_RIGHT = 7
  WALL_NO_SHIFT_LEFT = 8
  NARROW = 9


# a cut toward a real boundary; NARROW moves are fast only when urgent, a wall freeze never moves
_FAST_CAUSES = (LimitCause.RIGHT_LINE, LimitCause.LEFT_LINE, LimitCause.RIGHT_EDGE, LimitCause.LEFT_EDGE,
                LimitCause.RIGHT_WALL, LimitCause.LEFT_WALL)


@dataclass
class Slack:
  """Per side: smallest planned body clearance minus that boundary's margin (m, None = no
  boundary seen), which boundary it is, and whether a radar wall is within WALL_NEAR."""
  right: float | None = None
  left: float | None = None
  right_cause: LimitCause = LimitCause.NONE
  left_cause: LimitCause = LimitCause.NONE
  wall_right: bool = False
  wall_left: bool = False
  wall_near_right: bool = False
  wall_near_left: bool = False


def plan_slack(model_v2, v_ego: float, camera_offset: float, half_width: float, margin: float,
               walls=(None, None), wall_margin: float = DEFAULT_WALL_CLEARANCE) -> Slack:
  out = Slack()
  try:
    pos = model_v2.position
    px = np.asarray(pos.x, dtype=float)
    if px.size < 2:
      return out
    x = np.clip(np.array(GUARD_HORIZON) * max(float(v_ego), 0.0), 3.0, min(40.0, float(px[-1])))
    track = _interp(x, pos)
    probs, edge_stds = model_v2.laneLineProbs, model_v2.roadEdgeStds
    lines, edges = model_v2.laneLines, model_v2.roadEdges
    right: list[tuple[float, LimitCause]] = []
    left: list[tuple[float, LimitCause]] = []
    # model boundaries are in the shifted frame: real = model + camera offset
    if len(lines) >= 3 and len(probs) >= 3:
      if probs[2] >= LINE_MIN_PROB:
        right.append((np.min(_interp(x, lines[2]) + camera_offset - track) - half_width - margin, LimitCause.RIGHT_LINE))
      if probs[1] >= LINE_MIN_PROB:
        left.append((np.min(track - _interp(x, lines[1]) - camera_offset) - half_width - margin, LimitCause.LEFT_LINE))
    if len(edges) >= 2 and len(edge_stds) >= 2:
      if edge_stds[1] <= EDGE_MAX_STD:
        right.append((np.min(_interp(x, edges[1]) + camera_offset - track) - half_width - margin - EDGE_EXTRA, LimitCause.RIGHT_EDGE))
      if edge_stds[0] <= EDGE_MAX_STD:
        left.append((np.min(track - _interp(x, edges[0]) - camera_offset) - half_width - margin - EDGE_EXTRA, LimitCause.LEFT_EDGE))
    # radar walls are in the real frame already
    wl, wr = walls
    if wr is not None:
      out.wall_right = True
      gap = wr.at(x) - track - half_width
      right.append((float(np.min(gap)) - wall_margin, LimitCause.RIGHT_WALL))
      out.wall_near_right = bool(np.min(gap) < WALL_NEAR)
    if wl is not None:
      out.wall_left = True
      gap = track - wl.at(x) - half_width
      left.append((float(np.min(gap)) - wall_margin, LimitCause.LEFT_WALL))
      out.wall_near_left = bool(np.min(gap) < WALL_NEAR)
    right = [r for r in right if np.isfinite(r[0])]
    left = [lf for lf in left if np.isfinite(lf[0])]
    if right:
      out.right, out.right_cause = min(right, key=lambda r: r[0])
      out.right = float(out.right)
    if left:
      out.left, out.left_cause = min(left, key=lambda r: r[0])
      out.left = float(out.left)
  except (AttributeError, IndexError, TypeError, ValueError):
    pass
  return out


class ClearanceGuard:
  """Per-side limits on the lane offset (m, + = right) from the planned slack."""

  def __init__(self):
    self.reset()

  def reset(self) -> None:
    self.lim_right, self.lim_left = GUARD_OPEN, -GUARD_OPEN
    self.slack = Slack()
    self._ok_t = [0.0, 0.0]
    self.balance: float | None = None
    self._near_t = [WALL_HOLD, WALL_HOLD]  # s since a near wall was seen (right, left)
    self._freeze = [0.0, 0.0]  # offset when it appeared

  def set_slack(self, slack: Slack, offset_used: float = 0.0, dt: float = DT_MDL) -> None:
    """offset_used: the offset (m, + = right) the model frame behind `slack` was run with."""
    self.slack = slack
    if slack.right is None or slack.left is None:
      self.balance = None
      return
    # shifting right by d takes d off the right slack and adds it to the left
    b = float(np.clip(offset_used + 0.5 * (slack.right - slack.left + NARROW_BIAS), -NARROW_MAX, NARROW_MAX))
    if self.balance is None:
      self.balance = b
    else:
      self.balance += (1.0 - np.exp(-dt / NARROW_TAU)) * (b - self.balance)

  def _side(self, i: int, slack: float | None, lim: float, offset: float, sign: float, dt: float) -> float:
    # in "towards this side" coordinates the limit is an upper bound
    lim_s, off_s = sign * lim, sign * offset
    if slack is None:  # no boundary seen: hold
      self._ok_t[i] = 0.0
    elif slack < 0.0:
      self._ok_t[i] = 0.0
      lim_s = max(min(lim_s, off_s) - GUARD_RELEASE_RATE * dt, -KEEP_AWAY)
    elif slack > GUARD_HYST:
      self._ok_t[i] += dt
      if self._ok_t[i] >= GUARD_HOLD:
        lim_s = min(lim_s + OFFSET_RAMP_RATE * dt, GUARD_OPEN)
    else:
      self._ok_t[i] = 0.0
    return sign * lim_s

  def update(self, offset: float, dt: float = DT_CTRL) -> None:
    self.lim_right = self._side(0, self.slack.right, self.lim_right, offset, 1.0, dt)
    self.lim_left = self._side(1, self.slack.left, self.lim_left, offset, -1.0, dt)
    for i, near in enumerate((self.slack.wall_near_right, self.slack.wall_near_left)):
      if near:
        if self._near_t[i] >= WALL_HOLD:
          self._freeze[i] = offset
        self._near_t[i] = 0.0
      else:
        self._near_t[i] = min(self._near_t[i] + dt, WALL_HOLD)

  def urgent(self, step: float) -> bool:
    """True when moving the offset by `step` (+ = right) gets away from a side that is already
    inside its margin -- the only case a narrow-lane move needs the fast guard rate."""
    slack = self.slack.left if step > 0.0 else self.slack.right
    return slack is not None and slack < 0.0

  def apply(self, target: float) -> tuple[float, LimitCause]:
    if self.lim_left > self.lim_right:  # too close on both sides: balance the clearances
      if self.balance is not None:
        return self.balance, LimitCause.NARROW
      return 0.5 * (self.lim_left + self.lim_right), LimitCause.NARROW
    out, cause = target, LimitCause.NONE
    if target > self.lim_right:
      out, cause = self.lim_right, self.slack.right_cause
    elif target < self.lim_left:
      out, cause = self.lim_left, self.slack.left_cause
    # never shift toward a wall close to the body: freeze where the offset was when it appeared
    # (shifting away stays allowed)
    if self._near_t[0] < WALL_HOLD and out > max(self._freeze[0], 0.0):
      out, cause = max(self._freeze[0], 0.0), LimitCause.WALL_NO_SHIFT_RIGHT
    elif self._near_t[1] < WALL_HOLD and out < min(self._freeze[1], 0.0):
      out, cause = min(self._freeze[1], 0.0), LimitCause.WALL_NO_SHIFT_LEFT
    return out, cause


class DynamicLaneCentering:
  def __init__(self):
    self.enabled = False
    self.offset_mag = DEFAULT_OFFSET
    self.pause_on_signal = True
    self.e2e_authority = 1.0
    self.controller = LaneCenteringController()
    self.classifier = LanePositionClassifier()
    self.offset = 0.0  # applied (rate-limited) offset, m, positive = right
    self.offset_target = 0.0
    self.correction = 0.0  # curvature added to the model command, 1/m
    self.active = False
    self.mode = OffsetMode.CAMERA
    self.camera_offset = 0.0  # handed to modeld, m, CameraOffset convention (+ = left)
    self._camera_status = LaneCenteringStatus.OFF
    self._lines_ok = False
    self.guard = ClearanceGuard()
    self.margin = DEFAULT_MIN_CLEARANCE
    self.wall_margin = DEFAULT_WALL_CLEARANCE
    self.half_width = DEFAULT_CAR_WIDTH / 2
    self.limited = False
    self.limit_cause = LimitCause.NONE

  @property
  def position(self) -> LanePosition:
    return self.classifier.position

  @property
  def status(self) -> LaneCenteringStatus:
    if not self.enabled:
      return LaneCenteringStatus.OFF
    return self._camera_status if self.mode == OffsetMode.CAMERA else self.controller.status

  def get_params(self, params: Params) -> None:
    self.enabled = params.get_bool("LaneCentering")
    try:
      self.mode = OffsetMode(int(params.get("LaneCenteringOffsetMode", return_default=True)))
    except (TypeError, ValueError):
      self.mode = OffsetMode.CAMERA
    self.offset_mag = _get_float(params, "LaneCenterOffset", DEFAULT_OFFSET, 0.0, 0.3)
    self.pause_on_signal = params.get_bool("LaneCenteringPauseOnSignal")
    self.e2e_authority = _get_float(params, "LaneCenteringE2EAuthority", 1.0, 0.0, 1.0)
    self.controller.set_tuning(_get_float(params, "LaneCenteringGain", 0.30, 0.0, 1.0),
                               _get_float(params, "LaneCenteringSmoothTau", 0.4, 0.0, 5.0),
                               _get_float(params, "LaneCenteringDeadband", 0.08, 0.0, 0.5),
                               _get_float(params, "LaneCenteringReleaseHold", 0.5, 0.0, 5.0),
                               _get_float(params, "LaneCenteringReleaseTau", 1.0, 0.0, 10.0),
                               _get_float(params, "LaneCenteringReleaseJerk", 0.3, 0.0, 5.0))
    self.classifier.dwell = _get_float(params, "LaneCenteringDwell", DEFAULT_DWELL, 0.0, 30.0)
    self.classifier.switch_dist = _get_float(params, "LaneCenteringSwitchDist", DEFAULT_SWITCH_DIST, 0.0, 5000.0)
    self.margin = _get_float(params, "LaneCenteringMinClearance", DEFAULT_MIN_CLEARANCE, -0.5, 1.0)
    self.wall_margin = _get_float(params, "LaneCenteringWallClearance", DEFAULT_WALL_CLEARANCE, 0.0, 2.0)
    self.half_width = _get_float(params, "LaneCenteringCarWidth", DEFAULT_CAR_WIDTH, 1.0, 3.0) / 2

  def reset(self) -> None:
    self.controller.reset()
    self.classifier.reset()
    self.guard.reset()
    self.offset = self.offset_target = self.correction = self.camera_offset = 0.0
    self.active = self.limited = False
    self.limit_cause = LimitCause.NONE

  def _ramp_offset(self, rate: float = OFFSET_RAMP_RATE) -> None:
    step = rate * DT_CTRL
    self.offset += min(max(self.offset_target - self.offset, -step), step)

  def update(self, model_curvature: float, model_v2, model_updated: bool, model_valid: bool, v_ego: float,
             lat_active: bool, turn_signal: bool, driver_override: bool, applied_camera_offset: float | None = None,
             walls=(None, None)) -> float:
    """Call every controlsd frame (DT_CTRL). Returns the (possibly) corrected curvature."""
    if not self.enabled:
      if self.active or self.correction != 0.0 or self.position != LanePosition.UNKNOWN:
        self.controller.reset()
        self.classifier.reset()
        self.correction = 0.0
        self.active = False
      # the camera offset is a model input: ramp it out, never step it
      self.guard.reset()
      self.limited = False
      self.limit_cause = LimitCause.NONE
      self.offset_target = 0.0
      self._ramp_offset()
      self.camera_offset = -self.offset if self.mode == OffsetMode.CAMERA else 0.0
      return model_curvature

    if model_updated:
      self.classifier.update(model_v2, v_ego)
      try:
        probs = model_v2.laneLineProbs
        self._lines_ok = len(probs) >= 3 and min(probs[1], probs[2]) >= CAMERA_STATUS_MIN_PROB
      except (AttributeError, TypeError):
        self._lines_ok = False
    nominal = OFFSET_SIGN.get(self.classifier.position, 0.0) * self.offset_mag

    if self.mode == OffsetMode.CAMERA:
      if model_updated:
        try:
          lane_change = model_v2.meta.laneChangeState in (LaneChangeState.laneChangeStarting, LaneChangeState.laneChangeFinishing)
        except AttributeError:
          lane_change = False
        if lane_change:  # the lines are about to belong to another lane: start over there
          self.guard.reset()
        else:
          cam = applied_camera_offset if applied_camera_offset is not None else self.camera_offset
          self.guard.set_slack(plan_slack(model_v2, v_ego, cam, self.half_width, self.margin, walls, self.wall_margin), -cam)
      self.guard.update(self.offset)
      self.offset_target, cause = self.guard.apply(nominal)
      self.limited = abs(self.offset_target - nominal) > CAMERA_STATUS_SETTLED
      self.limit_cause = cause if self.limited else LimitCause.NONE
      fast = self.limited and (self.limit_cause in _FAST_CAUSES or
                               (self.limit_cause == LimitCause.NARROW and self.guard.urgent(self.offset_target - self.offset)))
      self._ramp_offset(GUARD_RELEASE_RATE if fast else OFFSET_RAMP_RATE)
      # modeld applies the offset; the model's lane lines and path are then in the shifted
      # frame, and the lane-position classifier only uses line-to-line distances
      self.camera_offset = -self.offset
      if self.controller.correction != 0.0 or self.correction != 0.0:
        self.controller.reset()
      self.correction = 0.0
      self.active = abs(self.offset) > 1e-3
      St = LaneCenteringStatus
      if not lat_active or v_ego < 5.0:
        self._camera_status = St.STANDBY
      elif self.limited:
        self._camera_status = St.LIMITED
      elif not self._lines_ok:
        self._camera_status = St.NO_LINES  # informational: the offset is kept
      elif abs(self.offset_target - self.offset) > CAMERA_STATUS_SETTLED:
        self._camera_status = St.SHIFT_RIGHT if self.offset_target > self.offset else St.SHIFT_LEFT
      elif abs(self.offset) < CAMERA_STATUS_CENTRED:
        self._camera_status = St.CENTERED
      else:
        self._camera_status = St.HOLD_RIGHT if self.offset > 0.0 else St.HOLD_LEFT
      return model_curvature

    self.offset_target = nominal
    self._ramp_offset()
    self.camera_offset = 0.0
    out = self.controller.update(model_curvature, model_v2, v_ego, True, self.offset, self.e2e_authority,
                                 lat_active, model_valid, self.pause_on_signal, turn_signal, driver_override,
                                 model_updated)
    self.correction = out - float(model_curvature)
    self.active = abs(self.correction) > 1e-9
    return out
