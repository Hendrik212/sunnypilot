"""
Lane-centre assist: a small, speed-gated position correction on the model's curvature
command, pulling the car back toward the lane centre the model itself reports.

Why: on tighter corners (R < 300 m, < 20 m/s) the model's own path runs inside the bend --
0.13-0.26 m inside on average and 0.7-1.5 m inside at the apex of R < 60 m corners on
routes 209/20c/20d (2026-09-14), riding the inner line -- while the torque controller
tracks the command to within +-5%. The lane position error is in the PLAN, not the
controller, so the fix is a position term on the command, not a gain change.

How: from modelV2.laneLines[1]/[2] the lane centre c(x) over x in [0, d] (d = v *
LOOKAHEAD_T clipped to [D_MIN, D_MAX]) is fitted with a quadratic, which splits it
into bumper offset c0, relative heading h (the linear term) and road curvature (the
quadratic term). Only offset and heading are used -- the road curvature is already in the
model's command. The correction is a pure-pursuit-style law on a virtual target point,
dk = G * 2 * (c0 + lambda * d * h) / d^2, with the heading weight lambda = ZETA * sqrt(2 / G):
with dk = G*2/d^2 * (y + lambda*d*y'/v) the closed loop is y'' + (2 G v lambda / d) y' +
(2 G v^2 / d^2) y = 0, zeta = lambda * sqrt(G / 2) for any G (plain pure pursuit,
lambda = 1, has zeta = sqrt(G / 2): a LOW gain is LESS damped, which the closed-loop test
caught). lambda is computed from the EFFECTIVE gain (G times the speed fade) so the
damping holds through the fade -- sized for the full G it drops to zeta 0.44 at 21 m/s and
overshoots 0.27 m in the sim. The heading term is a derivative and amplifies heading
noise (at lambda = 3.2, d = 20 m, 0.5 deg reads as 0.55 m of offset); the least-squares
fit over [0, d] and the output filter are what keep that in check. No deadband: the linear law is already
negligible at small offsets (6e-5 1/m at 0.1 m, 10 m/s) and a deadband edge produced a
0.1 m limit-cycle wobble in the closed-loop sim. The result is first-order filtered and
slew limited (the model's lane-line estimate can jump 0.5 m in one frame on corner entry;
that must not step the wheel), and capped in curvature and in lateral acceleration.

Gain: in the sim (kinematic bicycle, 0.35 s actuator delay, 0.8 m initial offset) G = 0.2
converges in ~4-6 s with <= 0.08 m overshoot at 8-18 m/s; G = 0.3 saturates K_MAX at
8 m/s, the heading winds up past the linear design and it overshoots 0.3 m. Start at 0.2.

Frame: in this stack modelV2 lane-line y is positive to the RIGHT and curvature is
positive for a RIGHT turn (latcontrol_torque carries the -VM.calc_curvature flip). So
centre > 0 = car left of centre -> positive dk -> steer right. Verified on route 209 seg 2:
a LEFT lane change moves leftY from -1.9 toward 0 with negative curvature.

Gates: lateral active, speed in [V_MIN, V_MAX] with a linear fade from V_FADE. The band is
where the bias is: on confident lines (both probs >= 0.6, driver-free, routes 209/20c/20d)
the median inside offset is 0.05 m at 7-15 m/s, 0.12 at 15-18, 0.36 at 18-21, 0.3 above;
the fade ends at 23 m/s to stay out of the 25-33 m/s band where the highway lane-keeping
loop is marginal (see the phase-budget notes). Both lane lines confident -- in the R < 60 m
hairpins on narrow roads the model has NO lane lines (probs 0.03-0.17, stds 2 m), so the
assist is off there by construction. Plausible lane width, not
in or just after a lane change, no blinker. Any gate false ramps the correction to zero
through the same slew limit rather than dropping it.

Param: LaneCentreGain (FLOAT, 0..1). 0.0 = off (default), identity on the command.
"""
import math

import numpy as np

from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL

LANE_CENTRE_LOOKAHEAD_T = 1.0  # s
LANE_CENTRE_D_MIN = 5.0  # m
LANE_CENTRE_D_MAX = 20.0  # m
LANE_CENTRE_V_MIN = 3.0  # m/s
LANE_CENTRE_V_FADE = 18.0  # m/s, full gain below this
LANE_CENTRE_V_MAX = 23.0  # m/s, zero gain above this
LANE_CENTRE_ZETA = 1.0  # closed-loop damping ratio the heading weight is set for
LANE_CENTRE_PROB_MIN = 0.6
LANE_CENTRE_WIDTH_MIN = 2.6  # m
LANE_CENTRE_WIDTH_MAX = 4.5  # m
LANE_CENTRE_LC_HOLDOFF_T = 2.0  # s after a lane change ends
LANE_CENTRE_FILTER_TAU = 0.3  # s
LANE_CENTRE_SLEW = 2e-3  # 1/m per s
LANE_CENTRE_K_MAX = 2.5e-3  # 1/m
LANE_CENTRE_LAT_ACCEL_MAX = 0.4  # m/s^2


class LaneCentreAssist:
  def __init__(self):
    self.gain = 0.0
    self.correction = 0.0  # filtered + slewed curvature correction, 1/m
    self.offset = 0.0  # bumper lane-centre offset, m (positive = car left of centre)
    self.active = False
    self.lc_holdoff = 0.0

  def get_params(self, params: Params) -> None:
    gain = params.get("LaneCentreGain", return_default=True)
    try:
      gain = float(gain) if gain is not None else 0.0
    except (TypeError, ValueError):
      gain = 0.0
    self.gain = max(0.0, min(1.0, gain))

  def reset(self) -> None:
    self.correction = 0.0
    self.offset = 0.0
    self.active = False
    self.lc_holdoff = 0.0

  @staticmethod
  def lane_centre(model_v2, d: float) -> tuple[float, float, float, bool]:
    """Returns (bumper offset c0, relative heading h [rad, positive = centre line veers
    right ahead], lane width, lines usable). Quadratic fit of the centre line over [0, d]."""
    probs = model_v2.laneLineProbs
    lines = model_v2.laneLines
    if len(lines) < 3 or len(probs) < 3:
      return 0.0, 0.0, 0.0, False
    left, right = lines[1], lines[2]
    if len(left.x) < 2 or len(right.x) < 2:
      return 0.0, 0.0, 0.0, False
    xl, yl = np.asarray(left.x, float), np.asarray(left.y, float)
    xr, yr = np.asarray(right.x, float), np.asarray(right.y, float)
    if len(xl) != len(xr) or len(xl) < 3:
      return 0.0, 0.0, 0.0, False
    # least-squares quadratic on the model's own (non-uniform) x grid up to d: exact for a
    # parabola, unlike interpolating 3 points, which leaks road curvature into the heading
    n = max(3, int(np.searchsorted(xl, d, side="right")))
    coef = np.polyfit(xl[:n], (yl[:n] + yr[:n]) / 2.0, 2)
    c0 = coef[2]
    h = coef[1]
    width = yr[0] - yl[0]
    usable = probs[1] >= LANE_CENTRE_PROB_MIN and probs[2] >= LANE_CENTRE_PROB_MIN and \
        LANE_CENTRE_WIDTH_MIN <= width <= LANE_CENTRE_WIDTH_MAX
    return float(c0), float(h), float(width), bool(usable)

  def update(self, lat_active: bool, v_ego: float, model_v2, lane_change_off: bool, blinker: bool) -> float:
    """Curvature correction (1/m) to ADD to the model command this frame. 0.0 when off."""
    if self.gain <= 0.0:
      self.reset()
      return 0.0

    if lane_change_off:
      self.lc_holdoff = max(self.lc_holdoff - DT_CTRL, 0.0)
    else:
      self.lc_holdoff = LANE_CENTRE_LC_HOLDOFF_T

    d = min(max(v_ego * LANE_CENTRE_LOOKAHEAD_T, LANE_CENTRE_D_MIN), LANE_CENTRE_D_MAX)
    c0, h, _, usable = self.lane_centre(model_v2, d)
    self.offset = c0

    speed_gain = float(np.interp(v_ego, [LANE_CENTRE_V_FADE, LANE_CENTRE_V_MAX], [1.0, 0.0]))
    gated = lat_active and usable and v_ego >= LANE_CENTRE_V_MIN and speed_gain > 0.0 and \
        self.lc_holdoff <= 0.0 and not blinker
    self.active = gated

    target = 0.0
    if gated:
      gain = self.gain * speed_gain
      lam = LANE_CENTRE_ZETA * math.sqrt(2.0 / gain)
      target = gain * 2.0 * (c0 + lam * d * h) / (d * d)
      k_max = min(LANE_CENTRE_K_MAX, LANE_CENTRE_LAT_ACCEL_MAX / max(v_ego, 1.0) ** 2)
      target = max(-k_max, min(k_max, target))

    alpha = 1.0 - math.exp(-DT_CTRL / LANE_CENTRE_FILTER_TAU)
    filtered = self.correction + alpha * (target - self.correction)
    step = LANE_CENTRE_SLEW * DT_CTRL
    self.correction = self.correction + max(-step, min(step, filtered - self.correction))
    return float(self.correction)
