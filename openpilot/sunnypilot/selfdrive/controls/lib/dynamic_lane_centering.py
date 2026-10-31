"""
Dynamic lane centering: StarPilot's LaneCenteringController (lane_centering.py) with the
lane offset chosen by which lane the car is in.

  RIGHT / SINGLE lane -> +offset (right of the lane centre)
  MIDDLE lane         -> -LaneCenteringMiddleBias (left of the centre, cancels the model's right bias)
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
Middle lane bias: route 25b (2026-10-31, segs 18-21, Autobahn ~96 km/h, small CD210 model) with
offset 0 in the MIDDLE lane put the car 0.08 m right of the lane centre and the plan 0.07 m right
of it at 20 m, no guard limit. The camera offset moves the car ~75% of the shift there (0.20 m
right offset in a RIGHT-lane stretch of the same drive: 0.08 -> 0.23 m), so 0.10 m left cancels it.

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
ahead -- the plan reacts at once to a camera shift, the car's current position would lag.
Routes 237/238 (2026-10-16): offset -0.25 on country right bends put the body within 10 cm
of the right line in 45% of frames (12% without offset), p5 -0.08 m.
Radar walls (radar_walls.py, radarWallsSP) are a third boundary with their own margin
(LaneCenteringWallClearance), and the offset never moves toward a wall within WALL_NEAR of
the body: route 238 seg 15 shifted 0.25 m toward a construction wall the model's road edge
put 0.3-0.6 m too far out, leaving the body 0.55-0.65 m from the concrete. A wall more than
WALL_BEYOND_LINE outside a confident lane line on its side is not "near" (it keeps its margin as
a boundary): route 24e (2026-10-24) froze the offset 22% of the country-road time on guardrails
and verges a median 0.78 m beyond the right line.

Each side's slack becomes a bound on the offset (hi = offset_used + right slack, lo =
offset_used - left slack). A bound tightens at once, holds BOUND_RELAX_HOLD, then relaxes over
BOUND_RELAX_TAU (a slight bend cuts the offset once instead of in and out every few seconds:
routes 24c/24e with an immediate 1.5 s relax and a fast cut on any line contact moved the offset
3.2-3.8 m/km, 35-45 reversals/km; replayed on 24a-24e this gives 17 instead of 31 reversals/km at
the same mean offset -- longer holds cut reversals further but eat the offset); the offset only
moves at GUARD_RELEASE_RATE when the plan is more than FAST_SLACK inside a margin, otherwise at
OFFSET_RAMP_RATE; a side
that stops showing a boundary keeps its bound for BOUND_HOLD (WALL_HOLD for a wall). The
offset is the nominal one clipped into [lo, hi]. When the window is empty (narrow lane) one
side gets its margin first: a near radar wall, else on a single-lane road the left (oncoming)
side; the other side's line or edge yields up to LINE_GIVE (WALL_GIVE against a wall, 0 when
the blind-spot monitor sees a car there), and what is still missing is split. Without a
priority (between two lanes of traffic) the deficit is split, NARROW_BIAS more room left.
Routes 248-24b (2026-10-20/22) with the previous latched limits + equal-slack balance:
- country roads sat in NARROW 23-39% of the time, in half of it with room on both sides (the
  latched limits regrew only after 2 s at 0.05 m/s), and the balance centred the car: body to
  the centre line 0.40 m in NARROW vs 0.67 m otherwise, offset ~0 instead of +0.06..+0.21;
- A560 construction zone: the model took the old white line as the right boundary (yellow
  lines further right), so both sides looked short; the balance hit its 0.30 m cap and the
  body was 0.35 m (p10) from the left wall while the driver steered right.
The offset is capped at OFFSET_CAP, WALL_CAP with a near wall (the plan follows ~half of a
camera shift).
A near wall FREEZES the offset where it was when the wall appeared (moving away stays allowed)
for WALL_HOLD s after it was last seen. Forcing the target to 0 instead stepped it 0.25 <-> 0
every 1-2 s along a flickering guardrail (route 23b, 36:56-37:41).

Params (all live, PARAMS_UPDATE_PERIOD): LaneCentering, LaneCenteringOffsetMode, LaneCenterOffset, LaneCenteringGain,
LaneCenteringPauseOnSignal, LaneCenteringSmoothTau, LaneCenteringDeadband, LaneCenteringDwell,
LaneCenteringSwitchDist, LaneCenteringE2EAuthority,
LaneCenteringReleaseHold, LaneCenteringReleaseTau, LaneCenteringReleaseJerk,
LaneCenteringMinClearance, LaneCenteringCarWidth, LaneCenteringWallClearance,
LaneCenteringMiddleBias. Frame: model y positive to the RIGHT.
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
DEFAULT_MIDDLE_BIAS = 0.10  # m left of the lane centre in the MIDDLE lane (+ = left)
WALL_NEAR = 1.5  # m, body to a radar wall: no offset toward it
WALL_BEYOND_LINE = 0.4  # m; a wall further than this outside a confident lane line is not near
GUARD_HORIZON = (0.5, 0.75, 1.0, 1.25, 1.5)  # s along the planned path
GUARD_RELEASE_RATE = 0.2  # m/s
BOUND_RELAX_HOLD = 1.0  # s a tightened bound holds before it relaxes
BOUND_RELAX_TAU = 3.0  # s, then it relaxes over this (it tightens at once)
FAST_SLACK = 0.05  # m inside a margin before the offset moves at GUARD_RELEASE_RATE
BOUND_HOLD = 2.0  # s a side keeps its bound after its boundary was last seen
WALL_HOLD = 3.0  # s a near wall keeps its bound and the offset frozen after it was last seen
OFFSET_CAP = 0.30  # m
WALL_CAP = 0.50  # m with a near wall: the plan only follows ~half a camera shift
NARROW_BIAS = 0.05  # m more clearance on the left than on the right when the deficit is split
LINE_GIVE = 0.25  # m a line or edge yields in a narrow lane (body ~at the paint at margin 0.25)
WALL_GIVE = 0.45  # m it yields against a near wall (construction lines are often the old ones)
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
  wall_slack_right: float | None = None  # the wall term alone
  wall_slack_left: float | None = None


def _wall_near(gap: float, line_gap: float | None) -> bool:
  """A wall within WALL_NEAR of the body that is the boundary -- not a guardrail or verge well
  beyond a confident lane line, which the line already keeps the car away from."""
  return gap < WALL_NEAR and (line_gap is None or gap < line_gap + WALL_BEYOND_LINE)


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
    line_gap = [None, None]  # body to a confident lane line (right, left)
    # model boundaries are in the shifted frame: real = model + camera offset
    if len(lines) >= 3 and len(probs) >= 3:
      if probs[2] >= LINE_MIN_PROB:
        line_gap[0] = float(np.min(_interp(x, lines[2]) + camera_offset - track) - half_width)
        right.append((line_gap[0] - margin, LimitCause.RIGHT_LINE))
      if probs[1] >= LINE_MIN_PROB:
        line_gap[1] = float(np.min(track - _interp(x, lines[1]) - camera_offset) - half_width)
        left.append((line_gap[1] - margin, LimitCause.LEFT_LINE))
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
      out.wall_slack_right = float(np.min(gap)) - wall_margin
      right.append((out.wall_slack_right, LimitCause.RIGHT_WALL))
      out.wall_near_right = _wall_near(float(np.min(gap)), line_gap[0])
    if wl is not None:
      out.wall_left = True
      gap = track - wl.at(x) - half_width
      out.wall_slack_left = float(np.min(gap)) - wall_margin
      left.append((out.wall_slack_left, LimitCause.LEFT_WALL))
      out.wall_near_left = _wall_near(float(np.min(gap)), line_gap[1])
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


class _Bound:
  """One side's bound on the offset in "towards this side" coordinates (an upper bound):
  tightens at once, holds BOUND_RELAX_HOLD, then relaxes over BOUND_RELAX_TAU; held for `hold` s once the side stops
  showing a boundary, then open (None)."""

  def __init__(self, hold: float):
    self.hold = hold
    self.reset()

  def reset(self) -> None:
    self.value: float | None = None
    self._age = 0.0
    self._since_tight = 0.0

  def update(self, raw: float | None, dt: float) -> None:
    if raw is None:
      self._age += dt
      if self._age >= self.hold:
        self.value = None
      return
    self._age = 0.0
    if self.value is None or raw < self.value:
      self.value = raw
      self._since_tight = 0.0
      return
    self._since_tight += dt
    if self._since_tight >= BOUND_RELAX_HOLD:
      self.value += (1.0 - np.exp(-dt / BOUND_RELAX_TAU)) * (raw - self.value)


class ClearanceGuard:
  """Feasible window [lo, hi] for the lane offset (m, + = right) from the planned slack."""

  def __init__(self):
    self._b = [_Bound(BOUND_HOLD), _Bound(BOUND_HOLD)]  # any boundary (right, left)
    self._w = [_Bound(WALL_HOLD), _Bound(WALL_HOLD)]  # radar walls only
    self.reset()

  def reset(self) -> None:
    for b in self._b + self._w:
      b.reset()
    self.slack = Slack()
    self._causes = [LimitCause.NONE, LimitCause.NONE]
    self._near_t = [WALL_HOLD, WALL_HOLD]  # s since a near wall was seen (right, left)
    self._freeze = [0.0, 0.0]  # offset when it appeared
    self.single = False  # single-lane road: the left side is oncoming traffic
    self.blindspot = (False, False)  # (right, left) car seen by the blind-spot monitor

  @property
  def hi(self) -> float | None:
    return self._b[0].value

  @property
  def lo(self) -> float | None:
    v = self._b[1].value
    return None if v is None else -v

  def set_slack(self, slack: Slack, offset_used: float = 0.0, dt: float = DT_MDL) -> None:
    """offset_used: the offset (m, + = right) the model frame behind `slack` was run with."""
    self.slack = slack
    # shifting right by d takes d off the right slack: bound = offset_used + slack, per side
    for i, sign, sl, wsl, cause in ((0, 1.0, slack.right, slack.wall_slack_right, slack.right_cause),
                                    (1, -1.0, slack.left, slack.wall_slack_left, slack.left_cause)):
      self._b[i].update(None if sl is None else sign * offset_used + sl, dt)
      self._w[i].update(None if wsl is None else sign * offset_used + wsl, dt)
      if sl is not None:
        self._causes[i] = cause

  def update(self, offset: float, dt: float = DT_CTRL) -> None:
    for i, near in enumerate((self.slack.wall_near_right, self.slack.wall_near_left)):
      if near:
        if self._near_t[i] >= WALL_HOLD:
          self._freeze[i] = offset
        self._near_t[i] = 0.0
      else:
        self._near_t[i] = min(self._near_t[i] + dt, WALL_HOLD)

  def urgent(self, step: float) -> bool:
    """True when moving the offset by `step` (+ = right) gets away from a side that is already
    more than FAST_SLACK inside its margin -- the only case a move needs the fast guard rate."""
    slack = self.slack.left if step > 0.0 else self.slack.right
    return slack is not None and slack < -FAST_SLACK

  def _give(self, i: int, wall_prio: bool) -> float:
    """How far side i may be crossed into its margin when the other side has priority."""
    if self.blindspot[i] or self._w[i].value is not None:
      return 0.0
    return WALL_GIVE if wall_prio else LINE_GIVE

  def apply(self, target: float) -> tuple[float, LimitCause]:
    near = [t < WALL_HOLD for t in self._near_t]
    cap = WALL_CAP if any(near) else OFFSET_CAP
    hi, lo = self.hi, self.lo
    out, cause = target, LimitCause.NONE
    if hi is not None and lo is not None and lo > hi:  # too close on both sides
      cause = LimitCause.NARROW
      if near[1] and not near[0]:
        prio = 1
      elif near[0] and not near[1]:
        prio = 0
      else:
        prio = 1 if self.single else None
      if prio == 1:  # left first: the right side yields
        hi_g = hi + self._give(0, near[1])
        out = hi_g + 0.5 * max(lo - hi_g, 0.0) if lo > hi_g else lo
      elif prio == 0:
        lo_g = lo - self._give(1, near[0])
        out = lo_g - 0.5 * max(lo_g - hi, 0.0) if lo_g > hi else hi
      else:
        out = 0.5 * (lo + hi + NARROW_BIAS)
    elif hi is not None and target > hi:
      out, cause = hi, self._causes[0]
    elif lo is not None and target < lo:
      out, cause = lo, self._causes[1]
    out = float(np.clip(out, -cap, cap))
    # never shift toward a wall close to the body: freeze where the offset was when it appeared
    # (shifting away stays allowed)
    if near[0] and out > max(self._freeze[0], 0.0):
      out, cause = max(self._freeze[0], 0.0), LimitCause.WALL_NO_SHIFT_RIGHT
    elif near[1] and out < min(self._freeze[1], 0.0):
      out, cause = min(self._freeze[1], 0.0), LimitCause.WALL_NO_SHIFT_LEFT
    return out, cause


class DynamicLaneCentering:
  def __init__(self):
    self.enabled = False
    self.offset_mag = DEFAULT_OFFSET
    self.middle_bias = DEFAULT_MIDDLE_BIAS
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
    self.middle_bias = _get_float(params, "LaneCenteringMiddleBias", DEFAULT_MIDDLE_BIAS, -0.2, 0.2)
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
             walls=(None, None), blindspot=(False, False)) -> float:
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
    if self.classifier.position == LanePosition.MIDDLE:
      nominal = -self.middle_bias
    else:
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
      self.guard.single = self.classifier.position == LanePosition.SINGLE
      self.guard.blindspot = (bool(blindspot[1]), bool(blindspot[0]))
      self.guard.update(self.offset)
      self.offset_target, cause = self.guard.apply(nominal)
      self.limited = abs(self.offset_target - nominal) > CAMERA_STATUS_SETTLED
      self.limit_cause = cause if self.limited else LimitCause.NONE
      step = self.offset_target - self.offset
      fast = self.limited and self.guard.urgent(step)
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
