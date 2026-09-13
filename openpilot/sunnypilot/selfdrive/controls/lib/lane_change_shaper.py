"""
Lane-change pace shaping. Ported from StarPilot's controlsd.py (Hendrik212/sunnypilot
98e0c3fc4, the `LaneChangeSmoothing` toggle, `LANE_CHANGE_ARREST_*`).

Why: with no shaping, a lane change runs at whatever pace the model commands. Measured on
route 00000209 (43 lane changes, 22-35 m/s): the model's own command peaks at 6-10 m/s^3
of lateral jerk on every one (above the 5.0 m/s^3 ISO clamp, so it rides the clamp), on a
quarter of them the sharpest kink is the ARREST (5.5-8.5 m/s^3, the moment the lane lines
re-assign and the plan flips to the new centre), and the CAN torque rate limiter is
engaged on 60-98% of frames for the whole manoeuvre. The driver reads that as "rough".

How: only the curvature-RATE clamp in clip_curvature is tightened, and only while the
model is in laneChangeStarting/Finishing. Lateral accel is left at the stock envelope --
capping it strangles the end-of-manoeuvre arrest and lets the car glide past the new lane
centre (StarPilot's stated reason). The tight clamp is held for the whole manoeuvre and
then released back to stock over LANE_CHANGE_SMOOTH_RELEASE_T so the model's recentre
step stays shaped instead of passing through a mostly-relaxed clamp.

The arrest is treated separately: entry gentleness is comfort, arrest speed is a
correctness constraint. When the model command moves AGAINST the entry direction, the
clamp is widened in proportion to how far the command lags the model (rate = lag / tau, a
P-pursuit) up to LANE_CHANGE_ARREST_JERK_FLOOR of stock. A boolean-gated fixed rate
engaged as a bang-bang switch right at the crest -- where the slow entry command meets a
model that has already peaked -- and snapped the wheel ~2 deg in 0.2 s (StarPilot's
lanechange1/2 rlogs, 2026-07-17); with the pursuit the lag is ~0 at the crest so the rate
crosses zero smoothly, and noise-scale lag inside the deadband gets no boost.

The pace (1..10) sets the entry clamp from a sinusoidal lane-change profile:
j = pi^3 * W / T^3 with W = 3.5 m, T = 3 + (10 - pace) * 5/9 s, x1.3 headroom, as a
fraction of the 5.0 m/s^3 stock clamp. Pace 10 is stock (factor 1.0, this module is
inert); pace 5 (StarPilot's default) targets ~5.8 s and clamps at 0.73 m/s^3.

Param: LaneChangeSmoothing (INT, 1..10). Default 10 = off, same convention as
LatLookaheadOffset (stock unless set).
"""
import math

from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import MAX_LATERAL_JERK

# After a smoothed lane change ends, ramp the curvature limits back to stock over this
# time so the final recenter correction is shaped instead of stepping through unclamped.
LANE_CHANGE_SMOOTH_RELEASE_T = 2.0  # s

# Cap on the extra jerk factor granted while the model is unwinding lane-change curvature
# (the arrest and any correction back toward center). 0.6 (~ pace-5 rate) fully tracks
# the arrest demand seen in logs, 40% below stock.
LANE_CHANGE_ARREST_JERK_FLOOR = 0.6
LANE_CHANGE_ARREST_PURSUIT_TAU = 0.2   # s
LANE_CHANGE_ARREST_GAP_DEADBAND = 5e-5  # 1/m
LANE_CHANGE_ARREST_RISE_TAU = 0.2  # s

# Entry-sign latch: the first command step larger than this after the manoeuvre starts
# defines the entry direction; steps against it are "unwinding".
LANE_CHANGE_ENTRY_STEP = 2e-4  # 1/m

LANE_CHANGE_PACE_STOCK = 10
LANE_CHANGE_LANE_WIDTH = 3.5  # m


def jerk_factor_for_pace(pace: int) -> float:
  """Entry jerk clamp as a fraction of MAX_LATERAL_JERK for a pace 1..10 (10 = stock)."""
  pace = max(1, min(LANE_CHANGE_PACE_STOCK, int(pace)))
  if pace >= LANE_CHANGE_PACE_STOCK:
    return 1.0
  t_target = 3.0 + (LANE_CHANGE_PACE_STOCK - pace) * 5.0 / 9.0
  j_req = (math.pi ** 3) * LANE_CHANGE_LANE_WIDTH / (t_target ** 3)
  return min(1.0, j_req * 1.3 / MAX_LATERAL_JERK)


class LaneChangeShaper:
  def __init__(self):
    self.pace = LANE_CHANGE_PACE_STOCK
    self.set_jerk = 1.0
    self.smooth_release = 0.0
    self.entry_sign = 0.0
    self.arrest_jerk_factor = 1.0

  def get_params(self, params: Params) -> None:
    pace = params.get("LaneChangeSmoothing", return_default=True)
    try:
      pace = int(pace) if pace is not None else LANE_CHANGE_PACE_STOCK
    except (TypeError, ValueError):
      pace = LANE_CHANGE_PACE_STOCK
    self.pace = max(1, min(LANE_CHANGE_PACE_STOCK, pace))
    self.set_jerk = jerk_factor_for_pace(self.pace)

  def reset(self) -> None:
    self.smooth_release = 0.0
    self.entry_sign = 0.0
    self.arrest_jerk_factor = 1.0

  def update(self, in_lane_change: bool, v_ego: float, new_desired_curvature: float,
             prev_desired_curvature: float) -> float:
    """Jerk factor for clip_curvature this frame. 1.0 = stock clamp.

    in_lane_change: model laneChangeState in (laneChangeStarting, laneChangeFinishing).
    prev_desired_curvature: the clipped command of the previous frame (controlsd's
    self.desired_curvature), so `step` is what clip_curvature is about to be asked for.
    """
    if self.pace >= LANE_CHANGE_PACE_STOCK:
      self.reset()
      return 1.0

    if in_lane_change:
      self.smooth_release = LANE_CHANGE_SMOOTH_RELEASE_T
      if self.entry_sign == 0.0 and abs(new_desired_curvature - prev_desired_curvature) > LANE_CHANGE_ENTRY_STEP:
        self.entry_sign = math.copysign(1.0, new_desired_curvature - prev_desired_curvature)
    else:
      self.smooth_release = max(self.smooth_release - DT_CTRL, 0.0)
      if self.smooth_release <= 0.0:
        self.entry_sign = 0.0

    jerk_factor = 1.0
    if self.smooth_release > 0.0:
      release = 1.0 - self.smooth_release / LANE_CHANGE_SMOOTH_RELEASE_T  # 0 in manoeuvre -> 1 after
      jerk_factor = self.set_jerk + (1.0 - self.set_jerk) * release
      step = new_desired_curvature - prev_desired_curvature
      model_unwinding = self.entry_sign != 0.0 and abs(step) > LANE_CHANGE_ARREST_GAP_DEADBAND and \
          math.copysign(1.0, step) == -self.entry_sign
      if model_unwinding:
        v_lim = max(v_ego, 1.0)
        gap = max(abs(step) - LANE_CHANGE_ARREST_GAP_DEADBAND, 0.0)
        jf_gap = (gap / LANE_CHANGE_ARREST_PURSUIT_TAU) * v_lim ** 2 / MAX_LATERAL_JERK
        arrest_cap = LANE_CHANGE_ARREST_JERK_FLOOR + (1.0 - LANE_CHANGE_ARREST_JERK_FLOOR) * release
        jerk_factor = max(jerk_factor, min(arrest_cap, jerk_factor + jf_gap))
    # Any widening of the clamp is rise-filtered; tightening is immediate.
    if jerk_factor > self.arrest_jerk_factor:
      rise_alpha = 1.0 - math.exp(-DT_CTRL / LANE_CHANGE_ARREST_RISE_TAU)
      jerk_factor = self.arrest_jerk_factor + rise_alpha * (jerk_factor - self.arrest_jerk_factor)
    self.arrest_jerk_factor = jerk_factor
    return float(jerk_factor)
