"""
Dynamic lane centering: StarPilot's LaneCenteringController (lane_centering.py) with the
lane offset chosen by which lane the car is in.

  RIGHT / SINGLE lane -> +offset (right of the lane centre)
  MIDDLE lane         -> -LaneCenteringMiddleBias (left of the centre, cancels the model's right bias)
  LEFT lane           -> -LaneCenterOffsetLeft, or -offset while it is negative (left of the lane centre)
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
Route 25d (2026-11-02, A560 construction zone, wall clearance 0.5, lane offset 0.20): the zone
reads as two lanes, so the LEFT-lane offset -0.20 ran next to the left wall -- frozen there
because it was already applied when the wall appeared -- body 0.39-0.56 m from the left wall and
1.1-1.5 m from the right one; the wall slack 10-30 m ahead saw more room than at the car. Where
only the left wall remained the model's left line was the old one ~1 m behind the concrete, the
narrow-lane push hit WALL_CAP 0.5 (the plan follows ~half), body median 0.34 m / min 0.19 m from
the wall, the driver steering right 69% of the time. So:
- a near wall takes the offset toward it back to 0 (not frozen where it was);
- near / inside / two-wall detection also takes the wall gap at the car (x = 0). The slack (the
  bound) stays on the plan: a bound is offset_used + slack, and the gap at the car only follows
  the offset late and by ~half, so it ratchets the bound away from the wall up to the cap;
- with a wall within WALL_NEAR on each side (no line filter: two walls are a construction channel)
  the target is the middle between them, relative to the lane centre, held WALL_HOLD;
- a near wall inside the model's line on its side (the line is not the boundary) raises the
  cap to WALL_INSIDE_CAP and the other side's give to WALL_INSIDE_GIVE.
Route 266 (2026-11-03), same zone with that in place: between two walls the car kept within
0.2 m of their middle. With only the left wall, outbound, the offset sat at the 0.8 m cap and
the real plan still pointed 0.5 m left (the model held to the old line behind the wall), body
0.30-0.40 m from the wall, driver steering right. On the way back the wall was 0.9-1.3 m off and
the no-shift rule held the offset at 0 while the car rode 0.1-0.3 m from the right line (the
model ran 0.2-0.4 m right of its centre, left line barely seen). So:
- one near wall and a confident line on the other side: the target is their middle, measured
  against the plan and added to the offset in use, so the offset keeps moving until the plan is
  there (the plan follows only part of a shift); wall clearance and the line's give still bound it;
- WALL_INSIDE_CAP 0.8 -> 1.2 m, live as LaneCenteringWallInsideCap; it also caps a middle
  target (WALL_CAP stopped the plan half way there).
Route 272 (2026-11-09, same zone, ~35 s between two walls): the middle target was taken raw from
every model frame and swung up to 0.33 m in 0.5 s -- the left wall was fitted only about a third
of the time and the left line sat around LINE_MIN_PROB, so it switched between the two-wall
middle (against the lane centre) and the wall-line middle (against the plan); the offset
wandered -0.15..+0.26 m and the car drifted left and right. So both middles are measured against
the plan (the two-wall one needs no lane line) and low-passed over MID_TAU (a deadband would leave
up to its width as a lasting error: the middle is integrated against the plan).
Route 269 (2026-11-04, construction zone ~75 km/h, ~3 min): a 2.70 m lane, on the left the old
white line and the valid yellow one between it and the guardrail (road edge 0.7-1.2 m beyond
the white line). The model's lines were the white one, its plan rode the yellow lane ~0.15 m
from the white paint; the 0.25 m margin pushed the offset from -0.10 to the +0.30 cap and the
driver steered left 20-50% of the time. So in a lane narrower than BARRIER_LANE_WIDTH with a
lane on the other side (MIDDLE/LEFT), above BARRIER_MIN_SPEED, a line with a road edge or radar
wall BARRIER_BEYOND outside it gets BARRIER_LINE_MARGIN; the barrier keeps its own clearance.
On routes 24f-26a this fires only there (2.9 min) and for seconds elsewhere; RIGHT/SINGLE
country roads are excluded (a verge behind the right line is common there, routes 237/238).
Route 275 (2026-11-09, L352 hill, two-lane road, 35-55 km/h, Cheddar): the plan itself rides
0.27-0.33 m inside every bend (KV and CD210 the same), the left body gap in left bends was below
-0.06 m in 5% of the frames (on the centre line). The oncoming lane reads as a lane, so the road
was RIGHT, not SINGLE, the left line had the plain 0.15 m margin, and a guardrail on the right
(a near radar wall) took the narrow-lane priority and the no-shift rule and pushed the offset
to -0.3..-0.5 m before and in left bends. So below ONCOMING_SPEED in a RIGHT or SINGLE lane the
left side counts as oncoming traffic: the left line gets LaneCenteringOncomingClearance, a
guardrail on the right no longer takes the priority or the no-shift rule (it keeps its own
clearance), and the cap to the right is LaneCenteringOncomingCap. (Until 2026-11-13 a narrow lane
kept the left margin in full and the right side yielded all of it: see routes 278-27a below.) A near wall on
the left, or a car in the right blind spot, keeps the rules above. Closed-loop replay (plan
follows 0.5 of a shift) on 275: left-bend gap p5 -0.06 -> +0.06 m, right-bend right gap p5
+0.08 -> +0.07, offset travel +12-16%/km; above 80 km/h and in the A560 zone (272) unchanged.
Routes 278/279/27a (2026-11-13, country roads with 2.2-2.6 m lanes, 6.8 min engaged): with that rule
the window was empty 56% of the time (the plan cuts every bend by 0.3-0.65 m, and a bound tightens
at once but relaxes over seconds, so on a winding road both sides look short), the right side
yielded without limit while it was 0.3 m inside its margin, and the offset sat above 0.3 m 31% of
the time (cap 0.45; up to 0.93 where a guardrail up to 0.4 m behind the right line made the target
the middle between it and the left line). The body was over the right line in 5% of the frames by
0.19 m or more, 0.06 m over the verge, with the driver steering left 26-28% of the time. So with
oncoming traffic an empty window is split evenly, a lone wall on the right gives no middle target
(the road ends at its line, unlike a construction lane), and the cap to the right is
LaneCenteringOncomingCap, now 0.30, also next to a guardrail (WALL_CAP and the inside cap are for
moving away from a wall). The centre line in left bends is the bend trim's job now
(modeld_v2/bend_trim.py). Closed-loop replay with this module and the bend trim in the loop (the
model as a 0.49 m/s^2 per m spring), these routes: offset above 0.3 m 31% -> 0%, body to the right
line p5 -0.19 -> -0.01 m (+0.02 with the trim at 1.0), to the verge -0.06 -> +0.12 (+0.14), to the
left line +0.16 -> +0.01 (+0.05). Only 12% of that time was free of driver torque, so the
positions are weak evidence; the offsets are not. Routes 275-277, worst body gap to the centre
line in left bends (R 40-80 / 80-120 / 120-250 m): -0.05/-0.06/-0.17 before, -0.12/-0.15/-0.30
with this change alone, +0.10/+0.01/-0.12 with the trim at 1.0 (+0.03/-0.01/-0.15 at 0.5): without
the trim the left bends lose what the rule gave.
Routes 27f/280 (2026-11-18, the same kind of road, lanes 2.2-2.6 m, NARROW 63-69%): the offset target
swung -0.30..+0.30 m (-0.50 by a guardrail), 8 reversals a minute, the body over a line 8-15% of the
time, the driver steering against it 17-28%. The narrow middle is offset_used + half the plan's slack
difference, and the plan follows only part of a shift and seconds late (target follows the car's
position 0.5-1 s later with the opposite sign, the car follows the target 3.5-9.5 s later), so the
middle keeps adding up and the ramp runs it from cap to cap. LaneCenteringNarrowMode 1 measures both
sides from the lane centre between the two confident lines instead (lane_right/lane_left, no
offset_used): an empty window there is split with the same priorities, and when only the plan is
pinched (a cut bend) the target is clipped into the lane window. Closed-loop replay of 27f/280:
offset p5/p95 -0.30/+0.30 -> 0.00/+0.22, travel 3.0 -> 1.3 m/min; but the right-line gap p5 -0.05 ->
-0.30 m, since nothing follows the model's own drift any more (it rides right of centre there). The
positions replay the driver's hands (94% of the time), aimed at the old behaviour, so they cannot
decide it: mode 0 (the old window) is the default until a drive compares them.
Not addressed: the driver holding the car to one side moves the plan with it, and the window, so
the offset follows the hand (driver_override is not used in camera mode).
Route 28f (2026-11-23, highway, offset 0.15 back on): the offset moved 0.55 m/min above 80 km/h
(0.08 on 28e), all of it the guard (right line in gentle bends 33 %, median wall 25 %, narrow 24 %),
and the wheel swung 1.4-2.8 deg in those windows (0.5 deg otherwise). Closed-loop replay (opsim
lc/sim6.py, logged offset reproduced to 4 mm):
- in a construction zone with yellow lines and a barrier on the left the wall-line middle, integrated
  against a plan that kept pointing 0.2-0.3 m right, ran the offset to -0.65 m toward the barrier and
  then +0.48 m, the driver steering against both. The middle is now clipped into the lane-centre
  window (lines and walls with their margins); clearance from the line opposite stays the guard's
  job. Zone offset p5 -0.43 -> -0.17 m; route 272 (A560 zone) and the country routes unchanged.
- right after a lane change the new lane's plan is still swinging in and the freshly reset bounds
  clamped on it (4 of the 20 largest moves): for LC_SETTLE a line or road-edge bound only acts when
  the plan is SETTLE_URGENT inside its margin. Offset-follow accel -7 %, no clearance cost.
Slower guard moves, longer bound holds and lane-centre bounds on the highway took another ~20 % off
but let the body 7 cm nearer the right line (p1): not done. The rest is the guard reading a plan
that cuts bends and follows a shift late.

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
The offset is capped at OFFSET_CAP, WALL_CAP with a near wall, WALL_INSIDE_CAP when that wall
stands inside the model's line (the plan follows ~half of a camera shift).
A near wall holds the offset toward it at 0 (moving away stays allowed) for WALL_HOLD s after it
was last seen; the hold and the rate-limited ramp keep a flickering wall from stepping the
target (route 23b, 36:56-37:41 stepped 0.25 <-> 0 every 1-2 s with no hold and no line filter).

Params (all live, PARAMS_UPDATE_PERIOD): LaneCentering, LaneCenteringOffsetMode, LaneCenterOffset, LaneCenterOffsetLeft,
LaneCenteringGain,
LaneCenteringPauseOnSignal, LaneCenteringSmoothTau, LaneCenteringDeadband, LaneCenteringDwell,
LaneCenteringSwitchDist, LaneCenteringE2EAuthority,
LaneCenteringReleaseHold, LaneCenteringReleaseTau, LaneCenteringReleaseJerk,
LaneCenteringMinClearance, LaneCenteringCarWidth, LaneCenteringWallClearance,
LaneCenteringMiddleBias, LaneCenteringWallInsideCap, LaneCenteringOncomingClearance,
LaneCenteringOncomingCap, LaneCenteringNarrowMode. Frame: model y positive to the RIGHT.
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
LC_SETTLE = 4.0  # s after a lane change a line or road-edge bound only acts when urgent (route 28f)
SETTLE_URGENT = 0.15  # m inside the margin: urgent
BOUND_HOLD = 2.0  # s a side keeps its bound after its boundary was last seen
WALL_HOLD = 3.0  # s a near wall keeps its bound and the offset frozen after it was last seen
MID_TAU = 3.0  # s low-pass on the middle target between walls (route 272: raw it swung 0.33 m in 0.5 s)
OFFSET_CAP = 0.30  # m
WALL_CAP = 0.50  # m with a near wall: the plan only follows ~half a camera shift
WALL_INSIDE_CAP = 1.20  # m when that wall stands inside the model's lane line on its side (LaneCenteringWallInsideCap)
WALL_INSIDE_GIVE = 0.70  # m the other side yields then (its line is likely an old one as well)
NARROW_BIAS = 0.05  # m more clearance on the left than on the right when the deficit is split
LINE_GIVE = 0.25  # m a line or edge yields in a narrow lane (body ~at the paint at margin 0.25)
WALL_GIVE = 0.45  # m it yields against a near wall (construction lines are often the old ones)
LINE_MIN_PROB = 0.5
# narrow multi-lane lane with a barrier just behind a line: the line is likely an old white one
# with the valid yellow one between it and the barrier, so the car may come closer to it
BARRIER_LINE_MARGIN = 0.05  # m, body to that line instead of the min clearance
BARRIER_LANE_WIDTH = 2.85  # m, lanes narrower than this (construction lanes 2.5-2.75 m)
BARRIER_BEYOND = (0.3, 1.2)  # m, road edge or radar wall this far outside the line
BARRIER_MIN_SPEED = 60 / 3.6  # m/s
ONCOMING_SPEED = (80 / 3.6, 85 / 3.6)  # m/s: below the first the left of a RIGHT/SINGLE lane is oncoming traffic, above the second no longer
DEFAULT_ONCOMING_CLEARANCE = 0.35  # m, body to the left line then (LaneCenteringOncomingClearance)
ONCOMING_CAP = 0.30  # m, offset cap to the right then (LaneCenteringOncomingCap); 0.45 until routes 278-27a
EDGE_MAX_STD = 0.5
# camera-mode status readout only
CAMERA_STATUS_MIN_PROB = 0.5
CAMERA_STATUS_SETTLED = 0.01  # m between applied and target offset
CAMERA_STATUS_CENTRED = 0.02  # m

DEFAULT_OFFSET = 0.15
# LEFT lane offset magnitude; negative = use LaneCenterOffset. Routes 28f/296 (Nov 22/24, offset 0.15): the
# model gives back most of a left camera shift (model-frame position ~0 cm, real -14/-16 cm, 70-72 cm wheel
# to left line) while it keeps the right one (real +26/+29 cm, 53-55 cm to the right line).
DEFAULT_OFFSET_LEFT = -1.0
OFFSET_LEFT_MAX = 0.40
# The RIGHT/SINGLE lane offset fades out in narrow lanes: routes 289-28b (Nov 21),
# country lanes ~2.5 m wide leave the 1.88 m car ~0.31 m per side when centred, and the +0.15 m right
# offset put it on the right line in most of the 41 driver corrections.
OFFSET_ZERO_WIDTH = 2.8  # m lane width (line centres at SAMPLE_X): no offset at or below
OFFSET_FULL_WIDTH = 3.2  # m: the full offset from here up
WIDTH_TAU = 1.0  # s low-pass on the measured lane width
NO_LINES_HOLD = 1.0  # s with neither inner line seen before the offset target fades to 0
DEFAULT_DWELL = 3.0
DEFAULT_SWITCH_DIST = 400.0

# Traffic-jam / emergency-corridor (Rettungsgasse) mode (LaneCenteringJamMode, live; JamShift below)
JAM_ON_SPEED = 30 / 3.6  # m/s: on below this
JAM_OFF_SPEED = 40 / 3.6  # m/s: off above this ...
JAM_OFF_HOLD = 3.0  # s ... held this long
JAM_RECENT_SPEED = 90 / 3.6  # m/s: AUTO only after driving at least this fast ...
JAM_RECENT_S = 600.0  # s ... within this long (keeps it out of town traffic)
JAM_CAP = 1.0  # m, offset cap in the mode (modeld: LANE_CENTRE_MAX_OFFSET)
LEGACY_CAMERA_CAP = 0.50  # m, camera offset cap outside the mode (modeld's cap before the mode)
JAM_EDGE_MARGIN = 0.15  # m, car body to the road edge, never less
JAM_RAMP = 0.10  # m/s toward the side and back after the mode
JAM_PLAN_MIN_SPEED = 3.0  # m/s: the plan-based guard only acts above this (a standing plan does not follow a shift)
DEFAULT_JAM_CLEARANCE = -0.10  # m, car body to the line on the shift side (- = over it); LaneCenteringJamClearance
JAM_CLEARANCE_RANGE = (-0.30, 0.50)


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
  BLINDSPOT_RIGHT = 10  # jam mode: a car in the blind spot on the shift side holds the offset
  BLINDSPOT_LEFT = 11


class JamMode(IntEnum):
  OFF = 0
  AUTO = 1  # left in a clear LEFT lane, right in a clear RIGHT lane, otherwise centred
  RIGHT = 2  # manual, until the jam ends
  LEFT = 3




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
  wall_inside_right: bool = False  # near wall closer than the model's line on that side
  wall_inside_left: bool = False
  mid_err: float | None = None  # plan to the middle of two near walls, or of a lone near wall and the line opposite (m, + = right)
  barrier_right: bool = False  # that line got BARRIER_LINE_MARGIN (narrow lane, barrier just behind it)
  barrier_left: bool = False
  # the same per-side bounds measured from the lane centre instead of the plan (m, an offset bound
  # directly, no offset_used), None unless both lane lines are confident
  lane_right: float | None = None
  lane_left: float | None = None


def _wall_near(gap: float, line_gap: float | None) -> bool:
  """A wall within WALL_NEAR of the body that is the boundary -- not a guardrail or verge well
  beyond a confident lane line, which the line already keeps the car away from."""
  return gap < WALL_NEAR and (line_gap is None or gap < line_gap + WALL_BEYOND_LINE)


def _barrier_beyond(line_y: np.ndarray, sign: float, edge_y: np.ndarray | None, wall_y: np.ndarray | None) -> bool:
  """A road edge or radar wall BARRIER_BEYOND outside a line (sign +1 right side, -1 left side)."""
  lo, hi = BARRIER_BEYOND
  return any(y is not None and lo <= float(np.median(sign * (y - line_y))) <= hi for y in (edge_y, wall_y))


def plan_slack(model_v2, v_ego: float, camera_offset: float, half_width: float, margin: float,
               walls=(None, None), wall_margin: float = DEFAULT_WALL_CLEARANCE, multi_lane: bool = False,
               left_margin: float | None = None, right_wall_middle: bool = True) -> Slack:
  """left_margin: the left line's margin when the left side is oncoming traffic (at least `margin`).
  right_wall_middle: a lone near wall on the right and the left line give a middle target (not on a
  two-way road: a guardrail just behind the right line is not the lane's boundary there)."""
  out = Slack()
  line_margin_l = margin if left_margin is None else max(margin, left_margin)
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
    bounds_r: list[tuple[np.ndarray, float]] = []  # boundary y (real frame) and its margin, for the lane-centre bounds
    bounds_l: list[tuple[np.ndarray, float]] = []
    centre = None
    # model boundaries are in the shifted frame: real = model + camera offset
    wl, wr = walls
    if len(lines) >= 3 and len(probs) >= 3:
      line_l, line_r = _interp(x, lines[1]) + camera_offset, _interp(x, lines[2]) + camera_offset
      if multi_lane and v_ego > BARRIER_MIN_SPEED and min(probs[1], probs[2]) >= LINE_MIN_PROB and \
         float(np.median(line_r - line_l)) < BARRIER_LANE_WIDTH:
        edge_ok = len(edges) >= 2 and len(edge_stds) >= 2
        edge = [_interp(x, edges[i]) + camera_offset if edge_ok and edge_stds[i] <= EDGE_MAX_STD else None for i in (1, 0)]
        out.barrier_right = _barrier_beyond(line_r, 1.0, edge[0], None if wr is None else wr.at(x))
        out.barrier_left = _barrier_beyond(line_l, -1.0, edge[1], None if wl is None else wl.at(x))
      if probs[2] >= LINE_MIN_PROB:
        line_gap[0] = float(np.min(line_r - track) - half_width)
        right.append((line_gap[0] - (BARRIER_LINE_MARGIN if out.barrier_right else margin), LimitCause.RIGHT_LINE))
        bounds_r.append((line_r, BARRIER_LINE_MARGIN if out.barrier_right else margin))
      if probs[1] >= LINE_MIN_PROB:
        line_gap[1] = float(np.min(track - line_l) - half_width)
        left.append((line_gap[1] - (BARRIER_LINE_MARGIN if out.barrier_left else line_margin_l), LimitCause.LEFT_LINE))
        bounds_l.append((line_l, BARRIER_LINE_MARGIN if out.barrier_left else line_margin_l))
      if probs[1] >= LINE_MIN_PROB and probs[2] >= LINE_MIN_PROB:
        centre = 0.5 * (line_l + line_r)
    if len(edges) >= 2 and len(edge_stds) >= 2:
      if edge_stds[1] <= EDGE_MAX_STD:
        edge_r = _interp(x, edges[1]) + camera_offset
        right.append((np.min(edge_r - track) - half_width - margin - EDGE_EXTRA, LimitCause.RIGHT_EDGE))
        bounds_r.append((edge_r, margin + EDGE_EXTRA))
      if edge_stds[0] <= EDGE_MAX_STD:
        edge_l = _interp(x, edges[0]) + camera_offset
        left.append((np.min(track - edge_l) - half_width - margin - EDGE_EXTRA, LimitCause.LEFT_EDGE))
        bounds_l.append((edge_l, margin + EDGE_EXTRA))
    # radar walls are in the real frame already; the car is at y = 0 now
    if wr is not None:
      out.wall_right = True
      gap = float(np.min(wr.at(x) - track)) - half_width
      out.wall_slack_right = gap - wall_margin
      right.append((out.wall_slack_right, LimitCause.RIGHT_WALL))
      bounds_r.append((wr.at(x), wall_margin))
      gap = min(gap, float(wr.at(0.0)) - half_width)
      out.wall_near_right = _wall_near(gap, line_gap[0])
      out.wall_inside_right = out.wall_near_right and line_gap[0] is not None and gap < line_gap[0]
    if wl is not None:
      out.wall_left = True
      gap = float(np.min(track - wl.at(x))) - half_width
      out.wall_slack_left = gap - wall_margin
      left.append((out.wall_slack_left, LimitCause.LEFT_WALL))
      bounds_l.append((wl.at(x), wall_margin))
      gap = min(gap, -float(wl.at(0.0)) - half_width)
      out.wall_near_left = _wall_near(gap, line_gap[1])
      out.wall_inside_left = out.wall_near_left and line_gap[1] is not None and gap < line_gap[1]
    # middles are measured against the plan (not the lane centre: the wall-side line is often
    # old or not seen), so both kinds of middle agree when a wall drops in and out
    if wl is not None and wr is not None and max(-wl.at(0.0), wr.at(0.0)) - half_width < WALL_NEAR:
      out.mid_err = float(np.mean(0.5 * (wl.at(x) + wr.at(x)) - track))
    elif len(lines) >= 3 and len(probs) >= 3:
      # one near wall and a confident line on the other side: the middle between them
      if wl is not None and wr is None and out.wall_near_left and probs[2] >= LINE_MIN_PROB:
        out.mid_err = float(np.mean(0.5 * (wl.at(x) + _interp(x, lines[2]) + camera_offset) - track))
      elif right_wall_middle and wr is not None and wl is None and out.wall_near_right and probs[1] >= LINE_MIN_PROB:
        out.mid_err = float(np.mean(0.5 * (wr.at(x) + _interp(x, lines[1]) + camera_offset) - track))
    if centre is not None:
      lr = [float(np.min(y - centre)) - half_width - m for y, m in bounds_r]
      ll = [float(np.min(centre - y)) - half_width - m for y, m in bounds_l]
      lr, ll = [v for v in lr if np.isfinite(v)], [v for v in ll if np.isfinite(v)]
      if lr and ll:
        out.lane_right, out.lane_left = min(lr), min(ll)
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
    self._l = [_Bound(BOUND_HOLD), _Bound(BOUND_HOLD)]  # any boundary, from the lane centre (right, left)
    self.inside_cap = WALL_INSIDE_CAP  # live: LaneCenteringWallInsideCap
    self.oncoming_cap = ONCOMING_CAP  # live: LaneCenteringOncomingCap
    self.narrow_from_lane = False  # live: LaneCenteringNarrowMode 1
    self.reset()

  def reset(self) -> None:
    for b in self._b + self._w + self._l:
      b.reset()
    self.slack = Slack()
    self._causes = [LimitCause.NONE, LimitCause.NONE]
    self._near_t = [WALL_HOLD, WALL_HOLD]  # s since a near wall was seen (right, left)
    self._inside_t = [WALL_HOLD, WALL_HOLD]  # s since a near wall inside the model's line was seen
    self._mid: float | None = None  # low-passed offset between two walls, or a wall and the line opposite
    self._mid_t = WALL_HOLD
    self._frame_mid: float | None = None
    self.single = False  # single-lane road: the left side has the narrow-lane priority
    self.oncoming = False  # below ONCOMING_SPEED in a RIGHT/SINGLE lane: the left side is oncoming traffic
    self.blindspot = (False, False)  # (right, left) car seen by the blind-spot monitor
    self.settling = False  # within LC_SETTLE of a lane change

  @property
  def hi(self) -> float | None:
    return self._b[0].value

  @property
  def lo(self) -> float | None:
    v = self._b[1].value
    return None if v is None else -v

  @property
  def lane_window(self) -> tuple[float, float] | None:
    """(lo, hi) offset window measured from the lane centre, None unless both sides are known."""
    hi, lo = self._l[0].value, self._l[1].value
    return None if hi is None or lo is None else (-lo, hi)

  def set_slack(self, slack: Slack, offset_used: float = 0.0, dt: float = DT_MDL) -> None:
    """offset_used: the offset (m, + = right) the model frame behind `slack` was run with."""
    self.slack = slack
    # shifting right by d takes d off the right slack: bound = offset_used + slack, per side
    for i, sign, sl, wsl, cause in ((0, 1.0, slack.right, slack.wall_slack_right, slack.right_cause),
                                    (1, -1.0, slack.left, slack.wall_slack_left, slack.left_cause)):
      self._b[i].update(None if sl is None else sign * offset_used + sl, dt)
      self._w[i].update(None if wsl is None else sign * offset_used + wsl, dt)
      self._l[i].update(slack.lane_right if i == 0 else slack.lane_left, dt)
      if sl is not None:
        self._causes[i] = cause
    # the middle is relative to the plan, which follows only part of a shift: adding it to the
    # offset in use keeps moving the offset until the plan is there (or the cap)
    self._frame_mid = None if slack.mid_err is None else offset_used + slack.mid_err

  def update(self, dt: float = DT_CTRL) -> None:
    sl = self.slack
    for t, flags in ((self._near_t, (sl.wall_near_right, sl.wall_near_left)),
                     (self._inside_t, (sl.wall_inside_right, sl.wall_inside_left))):
      for i, seen in enumerate(flags):
        t[i] = 0.0 if seen else min(t[i] + dt, WALL_HOLD)
    if self._frame_mid is not None:
      if self.mid is None:
        self._mid = self._frame_mid
      else:
        self._mid += (1.0 - np.exp(-dt / MID_TAU)) * (self._frame_mid - self._mid)
      self._mid_t = 0.0
    else:
      self._mid_t = min(self._mid_t + dt, WALL_HOLD)

  @property
  def mid(self) -> float | None:
    """Offset (m, + = right) that centres the car between two walls, or a wall and the line
    opposite, held WALL_HOLD."""
    return self._mid if self._mid_t < WALL_HOLD else None

  @property
  def near(self) -> list[bool]:
    """(right, left) a near wall was seen within WALL_HOLD."""
    return [t < WALL_HOLD for t in self._near_t]

  def urgent(self, step: float) -> bool:
    """True when moving the offset by `step` (+ = right) gets away from a side that is already
    more than FAST_SLACK inside its margin -- the only case a move needs the fast guard rate."""
    slack = self.slack.left if step > 0.0 else self.slack.right
    return slack is not None and slack < -FAST_SLACK

  def _give(self, i: int, wall_prio: bool) -> float:
    """How far side i may be crossed into its margin when the other side has priority."""
    if self.blindspot[i] or self._w[i].value is not None:
      return 0.0
    if wall_prio:
      return WALL_INSIDE_GIVE if self._inside_t[1 - i] < WALL_HOLD else WALL_GIVE
    return LINE_GIVE

  def apply(self, target: float) -> tuple[float, LimitCause]:
    near = self.near
    # a middle target is measured in the real frame and bounded by the wall clearance, so it
    # gets the inside cap as well: with WALL_CAP the plan stopped half way (it follows ~half)
    inside = self.mid is not None or any(t < WALL_HOLD for t in self._inside_t)
    cap = self.inside_cap if inside else WALL_CAP if any(near) else OFFSET_CAP
    hi, lo = self.hi, self.lo
    if self.settling:  # the new lane's plan is still swinging in: lines and edges only when urgent
      line_edge = (LimitCause.RIGHT_LINE, LimitCause.LEFT_LINE, LimitCause.RIGHT_EDGE, LimitCause.LEFT_EDGE)
      if self._causes[0] in line_edge and (self.slack.right is None or self.slack.right > -SETTLE_URGENT):
        hi = None
      if self._causes[1] in line_edge and (self.slack.left is None or self.slack.left > -SETTLE_URGENT):
        lo = None
    # oncoming traffic on the left (no wall there): a guardrail on the right does not take the
    # priority or the no-shift rule (route 275: it pushed the car onto the centre line); an empty
    # window is split evenly (routes 278-27a: left first put the body over the right line)
    onc = self.oncoming and not near[1] and not self.blindspot[0]
    wall_hi = self._w[0].value
    out, cause = target, LimitCause.NONE
    lane = self.lane_window
    narrow_plan = hi is not None and lo is not None and lo > hi
    in_lane = False
    if self.narrow_from_lane and lane is not None and (narrow_plan or lane[0] > lane[1]):
      # the narrow window from the lane centre: the plan-based one is offset_used + slack, and the
      # plan follows only part of a shift and seconds late, so its middle kept adding up and swung
      # the offset +-0.3 m (routes 27f/280)
      lo_l, hi_l = lane
      if lo_l <= hi_l:  # the lane has room, only the plan is pinched (a cut bend): stay in the lane window
        out, cause = float(np.clip(target, lo_l, hi_l)), LimitCause.NARROW
        narrow_plan, in_lane = False, True
      else:
        lo, hi, narrow_plan = lo_l, hi_l, True
    if narrow_plan:  # too close on both sides
      cause = LimitCause.NARROW
      if near[1] and not near[0]:
        prio = 1
      elif near[0] and not near[1]:
        prio = 0
      else:
        prio = 1 if self.single or self.oncoming else None
      if onc:
        out = 0.5 * (lo + hi)
      elif prio == 1:  # left first: the right side yields
        hi_g = hi + self._give(0, near[1])
        out = hi_g + 0.5 * max(lo - hi_g, 0.0) if lo > hi_g else lo
      elif prio == 0:
        lo_g = lo - self._give(1, near[0])
        out = lo_g - 0.5 * max(lo_g - hi, 0.0) if lo_g > hi else hi
      else:
        out = 0.5 * (lo + hi + NARROW_BIAS)
    elif in_lane:
      pass
    elif hi is not None and target > hi:
      out, cause = hi, self._causes[0]
    elif lo is not None and target < lo:
      out, cause = lo, self._causes[1]
    # to the right with oncoming traffic: LaneCenteringOncomingCap, also next to a guardrail (the
    # wall caps are for moving away from a wall; routes 278-27a sat at WALL_CAP toward one)
    cap_right = self.oncoming_cap if onc and self.mid is None else max(cap, self.oncoming_cap) if self.oncoming else cap
    out = float(np.clip(out, -cap, cap_right))
    # never shift toward a wall close to the body (shifting away stays allowed); between two
    # walls, or a wall and a confident line opposite, the target is their middle, which is the
    # only offset toward a wall
    if self.mid is None:
      if near[0] and out > 0.0 and onc:
        if cause != LimitCause.NARROW and wall_hi is not None and out > max(wall_hi, 0.0):
          out, cause = max(wall_hi, 0.0), LimitCause.RIGHT_WALL
      elif near[0] and out > 0.0:
        out, cause = 0.0, LimitCause.WALL_NO_SHIFT_RIGHT
      elif near[1] and out < 0.0:
        out, cause = 0.0, LimitCause.WALL_NO_SHIFT_LEFT
    return out, cause


class JamShift:
  """Traffic-jam / emergency-corridor mode (camera mode only): below JAM_ON_SPEED the car moves to
  one side of its lane instead of the centre, the body `clearance` from the line on that side
  (negative = over it), never closer than JAM_EDGE_MARGIN to the road edge.

  Side: LaneCenteringJamMode AUTO shifts left in a LEFT lane and right in a RIGHT lane (the
  classifier's debounced position; MIDDLE, SINGLE and UNKNOWN stay centred) and only after
  JAM_RECENT_SPEED within JAM_RECENT_S, so town traffic does not hug the kerb; RIGHT / LEFT force a
  side until the jam ends (reset_manual asks controlsd to put the param back to AUTO).
  The line on the shift side may only be crossed with a confident road edge on that side and no
  lane behind the line (position LEFT/RIGHT on that side, or the edge within EDGE_NO_LANE of the
  line); otherwise, and always in a MIDDLE lane, the body stops at the line.

  Target: lane-centre geometry (half the low-passed lane width - half the car - clearance), not
  the plan: a standing car's plan does not follow a camera shift, so offset_used + plan slack
  would ratchet up to the cap. Above JAM_PLAN_MIN_SPEED the plan's slack to the line, the road
  edge and radar walls bounds it as well (the plan follows a shift there). A car in the blind spot
  on the shift side holds the offset where it is."""

  def __init__(self):
    self.mode = JamMode.AUTO
    self.clearance = DEFAULT_JAM_CLEARANCE
    self._since_fast = JAM_RECENT_S
    self.reset_manual = False
    self.reset()

  def reset(self) -> None:
    self.active = False
    self.side = 0
    self._off_t = 0.0
    self._edge = _Bound(BOUND_HOLD)  # toward-side limit from the road edge, from the lane centre
    self._plan = _Bound(BOUND_HOLD)  # toward-side limit from the plan, offset_used + slack
    self.target = 0.0  # m, toward the side (>= 0)
    self.cause = LimitCause.NONE
    self.limited = False

  def _leave(self) -> None:
    if self.mode in (JamMode.RIGHT, JamMode.LEFT):
      self.reset_manual = True
    self.reset()

  def update_speed(self, v_ego: float, dt: float = DT_CTRL) -> None:
    v_ego = float(v_ego)
    self._since_fast = 0.0 if v_ego > JAM_RECENT_SPEED else min(self._since_fast + dt, JAM_RECENT_S)
    if self.mode == JamMode.OFF:
      if self.active:
        self.reset()
      return
    if not self.active:
      if v_ego < JAM_ON_SPEED and (self.mode != JamMode.AUTO or self._since_fast < JAM_RECENT_S):
        self.active, self._off_t = True, 0.0
      return
    self._off_t = self._off_t + dt if v_ego > JAM_OFF_SPEED else 0.0
    if self._off_t >= JAM_OFF_HOLD:
      self._leave()

  def choose_side(self, pos: LanePosition) -> int:
    if not self.active:
      side = 0
    elif self.mode == JamMode.RIGHT:
      side = 1
    elif self.mode == JamMode.LEFT:
      side = -1
    else:
      side = 1 if pos == LanePosition.RIGHT else -1 if pos == LanePosition.LEFT else 0
    if side != self.side:
      self.clear_bounds()
      self.target = 0.0
      self.side = side
    return side

  def clear_bounds(self) -> None:
    self._edge.reset()
    self._plan.reset()

  def update_frame(self, model_v2, v_ego: float, cam: float, pos: LanePosition, lane_width: float | None,
                   half_width: float, offset_used: float, current: float, wall_bound: float | None,
                   blindspot: bool) -> None:
    """Once per NEW model frame. cam: camera offset the frame was run with (real = model + cam);
    offset_used / current: offset (m, + = right) behind the frame / applied now; wall_bound: the
    guard's radar-wall bound toward the side (offset_used + slack), None without a wall."""
    s = self.side
    if s == 0:
      return
    cur = max(s * current, 0.0)
    try:
      lines, probs = model_v2.laneLines, model_v2.laneLineProbs
      edges, edge_stds = model_v2.roadEdges, model_v2.roadEdgeStds
      i_line, i_edge = (2, 1) if s > 0 else (1, 0)
      line_ok = len(lines) >= 3 and len(probs) >= 3 and probs[i_line] >= LINE_MIN_PROB
      y1, y2 = _sample(lines[1]) + cam, _sample(lines[2]) + cam
      if min(probs[1], probs[2]) >= LINE_MIN_PROB:
        centre = 0.5 * (y1 + y2)
      elif lane_width is not None and max(probs[1], probs[2]) >= LINE_MIN_PROB:
        centre = y1 + 0.5 * lane_width if probs[1] >= probs[2] else y2 - 0.5 * lane_width
      else:
        centre = None
      edge_y = _sample(edges[i_edge]) + cam if len(edges) >= 2 and len(edge_stds) >= 2 and edge_stds[i_edge] <= EDGE_MAX_STD else None
      if edge_y is not None and not np.isfinite(edge_y):
        edge_y = None
      self._edge.update(None if centre is None or edge_y is None else s * (edge_y - centre) - half_width - JAM_EDGE_MARGIN, DT_MDL)
      outer = pos == (LanePosition.RIGHT if s > 0 else LanePosition.LEFT)
      line_y = y2 if s > 0 else y1
      no_lane_behind = outer or (edge_y is not None and line_ok and s * (edge_y - line_y) < EDGE_NO_LANE)
      cross = self._edge.value is not None and pos != LanePosition.MIDDLE and no_lane_behind
      margin = self.clearance if cross else max(self.clearance, 0.0)
      # the plan: only while rolling (standing, it does not follow a shift)
      plan = None
      if v_ego > JAM_PLAN_MIN_SPEED:
        px = np.asarray(model_v2.position.x, dtype=float)
        if px.size >= 2:
          x = np.clip(np.array(GUARD_HORIZON) * float(v_ego), 3.0, min(40.0, float(px[-1])))
          track = _interp(x, model_v2.position)
          sl = []
          if line_ok:
            sl.append(float(np.min(s * (_interp(x, lines[i_line]) + cam - track))) - half_width - margin)
          if edge_y is not None:
            sl.append(float(np.min(s * (_interp(x, edges[i_edge]) + cam - track))) - half_width - JAM_EDGE_MARGIN)
          sl = [v for v in sl if np.isfinite(v)]
          if sl:
            plan = s * offset_used + min(sl)
      self._plan.update(plan, DT_MDL)
    except (AttributeError, IndexError, TypeError, ValueError):
      return

    if lane_width is None or not np.isfinite(lane_width):
      target, cause = cur, LimitCause.NONE  # no lane width yet: hold
    else:
      target, cause = 0.5 * lane_width - half_width - margin, LimitCause.NONE
    line_cause = LimitCause.RIGHT_LINE if s > 0 else LimitCause.LEFT_LINE
    limits = [(self._edge.value, LimitCause.RIGHT_EDGE if s > 0 else LimitCause.LEFT_EDGE),
              (self._plan.value, line_cause)]
    if v_ego > JAM_PLAN_MIN_SPEED:
      limits.append((wall_bound, LimitCause.RIGHT_WALL if s > 0 else LimitCause.LEFT_WALL))
    for lim, c in limits:
      if lim is not None and lim < target:
        target, cause = lim, c
    if blindspot and target > cur:
      target, cause = cur, LimitCause.BLINDSPOT_RIGHT if s > 0 else LimitCause.BLINDSPOT_LEFT
    self.target = float(np.clip(target, 0.0, JAM_CAP))
    self.cause = cause
    self.limited = cause != LimitCause.NONE


class DynamicLaneCentering:
  def __init__(self):
    self.enabled = False
    self.offset_mag = DEFAULT_OFFSET
    self.offset_mag_left: float | None = None  # None = offset_mag
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
    self.lane_width: float | None = None  # low-passed inner line-to-line width, m
    self._no_lines_time = 0.0
    self.guard = ClearanceGuard()
    self.margin = DEFAULT_MIN_CLEARANCE
    self.wall_margin = DEFAULT_WALL_CLEARANCE
    self.oncoming_margin = DEFAULT_ONCOMING_CLEARANCE
    self.oncoming = False
    self.half_width = DEFAULT_CAR_WIDTH / 2
    self.limited = False
    self.limit_cause = LimitCause.NONE
    self._since_lane_change = LC_SETTLE
    self.jam = JamShift()
    self._jam_return = False  # offset beyond LEGACY_CAMERA_CAP left over from the jam mode

  @property
  def jam_on(self) -> bool:
    return self.jam.active and self.jam.side != 0

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
    left = _get_float(params, "LaneCenterOffsetLeft", DEFAULT_OFFSET_LEFT, DEFAULT_OFFSET_LEFT, OFFSET_LEFT_MAX)
    self.offset_mag_left = left if left >= 0.0 else None
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
    self.guard.inside_cap = _get_float(params, "LaneCenteringWallInsideCap", WALL_INSIDE_CAP, 0.3, 1.5)
    self.oncoming_margin = _get_float(params, "LaneCenteringOncomingClearance", DEFAULT_ONCOMING_CLEARANCE, 0.0, 0.6)
    self.guard.oncoming_cap = _get_float(params, "LaneCenteringOncomingCap", ONCOMING_CAP, 0.0, 0.6)
    self.guard.narrow_from_lane = _get_float(params, "LaneCenteringNarrowMode", 0.0, 0.0, 1.0) >= 0.5
    self.half_width = _get_float(params, "LaneCenteringCarWidth", DEFAULT_CAR_WIDTH, 1.0, 3.0) / 2
    try:
      self.jam.mode = JamMode(int(_get_float(params, "LaneCenteringJamMode", float(JamMode.AUTO), 0.0, 3.0)))
    except ValueError:
      self.jam.mode = JamMode.AUTO
    self.jam.clearance = _get_float(params, "LaneCenteringJamClearance", DEFAULT_JAM_CLEARANCE, *JAM_CLEARANCE_RANGE)

  def reset(self) -> None:
    self.controller.reset()
    self.classifier.reset()
    self.guard.reset()
    self.offset = self.offset_target = self.correction = self.camera_offset = 0.0
    self.active = self.limited = self.oncoming = False
    self.limit_cause = LimitCause.NONE
    self.lane_width = None
    self._no_lines_time = 0.0
    self._since_lane_change = LC_SETTLE
    self.jam.reset()
    self._jam_return = False

  def _update_width(self, model_v2) -> None:
    """Per NEW modelV2 frame: low-pass the inner lane width (both inner lines confident; held
    otherwise) and time how long BOTH inner lines are gone."""
    try:
      probs = model_v2.laneLineProbs
      none_seen = max(probs[1], probs[2]) < CAMERA_STATUS_MIN_PROB
    except (AttributeError, IndexError, TypeError):
      none_seen = True
    self._no_lines_time = self._no_lines_time + DT_MDL if none_seen else 0.0
    if not self._lines_ok:
      return
    try:
      width = _sample(model_v2.laneLines[2]) - _sample(model_v2.laneLines[1])
    except (AttributeError, IndexError, TypeError, ValueError):
      return
    if not (np.isfinite(width) and 1.5 < width < 6.0):
      return
    if self.lane_width is None:
      self.lane_width = width
    else:
      self.lane_width += (width - self.lane_width) * DT_MDL / (WIDTH_TAU + DT_MDL)

  def width_scale(self) -> float:
    """0..1 share of the RIGHT/SINGLE lane offset allowed: 0 in narrow lanes and once no inner
    line has been seen for NO_LINES_HOLD."""
    if self.lane_width is None or self._no_lines_time > NO_LINES_HOLD:
      return 0.0
    return float(np.clip((self.lane_width - OFFSET_ZERO_WIDTH) / (OFFSET_FULL_WIDTH - OFFSET_ZERO_WIDTH), 0.0, 1.0))

  def _camera_clip(self, offset: float) -> float:
    """The camera offset handed to modeld: up to JAM_CAP for the jam mode (and while its offset
    ramps back), LEGACY_CAMERA_CAP otherwise -- modeld clipped there before the mode, so the
    wall caps above it behave as they did."""
    cap = JAM_CAP if self._jam_return or self.jam_on else LEGACY_CAMERA_CAP
    return float(np.clip(offset, -cap, cap))

  def _jam_frame(self, model_v2, v_ego: float, cam: float, pos: LanePosition, lat_active: bool, blindspot) -> None:
    """Per NEW model frame: the jam side and its target."""
    side = self.jam.choose_side(pos)
    if side == 0 or not lat_active:
      return
    i = 0 if side > 0 else 1
    wall = self.guard._w[i].value
    self.jam.update_frame(model_v2, v_ego, cam, pos, self.lane_width, self.half_width, -cam, self.offset, wall,
                          bool(blindspot[1] if side > 0 else blindspot[0]))

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
      self.jam.reset()
      self.limited = False
      self.limit_cause = LimitCause.NONE
      self.offset_target = 0.0
      self._ramp_offset(JAM_RAMP if self._jam_return else OFFSET_RAMP_RATE)
      self._jam_return = self._jam_return and abs(self.offset) > LEGACY_CAMERA_CAP
      self.camera_offset = -self._camera_clip(self.offset) if self.mode == OffsetMode.CAMERA else 0.0
      return model_curvature

    if model_updated:
      self.classifier.update(model_v2, v_ego)
      try:
        probs = model_v2.laneLineProbs
        self._lines_ok = len(probs) >= 3 and min(probs[1], probs[2]) >= CAMERA_STATUS_MIN_PROB
      except (AttributeError, TypeError):
        self._lines_ok = False
      self._update_width(model_v2)
    if self.classifier.position == LanePosition.MIDDLE:
      nominal = -self.middle_bias
    else:
      mag = self.offset_mag
      if self.classifier.position == LanePosition.LEFT and self.offset_mag_left is not None:
        mag = self.offset_mag_left
      nominal = OFFSET_SIGN.get(self.classifier.position, 0.0) * mag
      if self.classifier.position in (LanePosition.RIGHT, LanePosition.SINGLE):
        # country roads: no right offset in narrow lanes or blind (LEFT/MIDDLE: construction-zone tuning)
        nominal *= self.width_scale()

    if self.mode == OffsetMode.CAMERA:
      pos = self.classifier.position
      if pos in (LanePosition.RIGHT, LanePosition.SINGLE):
        self.oncoming = v_ego < ONCOMING_SPEED[1] if self.oncoming else v_ego < ONCOMING_SPEED[0]
      else:
        self.oncoming = False
      if model_updated:
        try:
          lane_change = model_v2.meta.laneChangeState in (LaneChangeState.laneChangeStarting, LaneChangeState.laneChangeFinishing)
        except AttributeError:
          lane_change = False
        if lane_change:  # the lines are about to belong to another lane: start over there
          self.guard.reset()
          self.jam.clear_bounds()
          self._since_lane_change = 0.0
        else:
          cam = applied_camera_offset if applied_camera_offset is not None else self.camera_offset
          multi_lane = pos in (LanePosition.MIDDLE, LanePosition.LEFT)
          # a near wall on the left: no oncoming traffic beside the car, the plain line margin
          left_margin = self.oncoming_margin if self.oncoming and not self.guard.near[1] else None
          self.guard.set_slack(plan_slack(model_v2, v_ego, cam, self.half_width, self.margin, walls, self.wall_margin,
                                          multi_lane, left_margin, not self.oncoming), -cam)
          self._jam_frame(model_v2, v_ego, cam, pos, lat_active, blindspot)
      self._since_lane_change = min(self._since_lane_change + DT_CTRL, LC_SETTLE)
      self.guard.settling = self._since_lane_change < LC_SETTLE
      self.guard.single = pos == LanePosition.SINGLE
      self.guard.oncoming = self.oncoming
      self.guard.blindspot = (bool(blindspot[1]), bool(blindspot[0]))
      self.guard.update()
      if self.guard.mid is not None:
        # the middle is integrated against the plan; it stays inside the window the lane centre
        # leaves (lines and walls with their margins) -- route 28f ran it to -0.65 m toward a barrier
        lane = self.guard.lane_window
        nominal = self.guard.mid if lane is None or lane[0] > lane[1] else float(np.clip(self.guard.mid, lane[0], lane[1]))
      self.jam.update_speed(v_ego)
      if not self.jam.active:
        self.jam.choose_side(pos)
      if self.jam_on and lat_active:
        s = self.jam.side
        self.offset_target = s * self.jam.target
        self.limited = self.jam.limited
        self.limit_cause = self.jam.cause
        self._jam_return = True
        # toward the side at JAM_RAMP; back from it fast when a limit tightened
        back = s * (self.offset_target - self.offset) < 0.0 and self.jam.limited
        self._ramp_offset(GUARD_RELEASE_RATE if back else JAM_RAMP)
      else:
        self.offset_target, cause = self.guard.apply(nominal)
        self.limited = abs(self.offset_target - nominal) > CAMERA_STATUS_SETTLED
        self.limit_cause = cause if self.limited else LimitCause.NONE
        step = self.offset_target - self.offset
        fast = self.limited and self.guard.urgent(step)
        self._ramp_offset(GUARD_RELEASE_RATE if fast else JAM_RAMP if self._jam_return else OFFSET_RAMP_RATE)
      self._jam_return = self._jam_return and (self.jam_on or abs(self.offset) > LEGACY_CAMERA_CAP)
      # modeld applies the offset; the model's lane lines and path are then in the shifted
      # frame, and the lane-position classifier only uses line-to-line distances
      self.camera_offset = -self._camera_clip(self.offset)
      if self.controller.correction != 0.0 or self.correction != 0.0:
        self.controller.reset()
      self.correction = 0.0
      self.active = abs(self.offset) > 1e-3
      St = LaneCenteringStatus
      if self.jam_on and lat_active and not self.limited:
        self._camera_status = St.JAM_RIGHT if self.jam.side > 0 else St.JAM_LEFT
      elif self.jam_on and lat_active:
        self._camera_status = St.LIMITED
      elif not lat_active or v_ego < 5.0:
        self._camera_status = St.STANDBY
      elif self.limited:
        self._camera_status = St.LIMITED
      elif not self._lines_ok:
        self._camera_status = St.NO_LINES  # the offset is kept for NO_LINES_HOLD, then fades out
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
