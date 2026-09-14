import math
from collections import deque

import numpy as np

from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.controls.lib import lane_centre_assist as lca
from openpilot.sunnypilot.selfdrive.controls.lib.lane_centre_assist import LaneCentreAssist


class _P:
  def __init__(self, v):
    self.v = v

  def get(self, key, return_default=False):
    assert key == "LaneCentreGain"
    return self.v


class _Line:
  def __init__(self, x, y):
    self.x = list(x)
    self.y = list(y)


class _Model:
  """modelV2 stand-in: lane lines as y = offset + heading*x + 0.5*curv*x^2 (y positive right)."""

  def __init__(self, centre=0.0, heading=0.0, curv=0.0, width=3.6, probs=(0.9, 0.9)):
    x = np.array(ModelConstants.X_IDXS)
    c = centre + heading * x + 0.5 * curv * x ** 2
    self.laneLines = [_Line(x, c), _Line(x, c - width / 2), _Line(x, c + width / 2), _Line(x, c)]
    self.laneLineProbs = [0.0, probs[0], probs[1], 0.0]


def _assist(gain):
  a = LaneCentreAssist()
  a.get_params(_P(gain))
  return a


def _settle(a, v, model, n=400, lat_active=True, lc_off=True, blinker=False):
  out = 0.0
  for _ in range(n):
    out = a.update(lat_active, v, model, lc_off, blinker)
  return out


def test_gain_zero_is_identity():
  a = _assist(0.0)
  for _ in range(50):
    assert a.update(True, 10.0, _Model(centre=1.0), True, False) == 0.0
  assert a.offset == 0.0 and not a.active


def test_bad_param_is_off():
  for v in (None, "", "abc", -1.0):
    a = LaneCentreAssist()
    a.get_params(_P(v))
    assert a.gain == 0.0


def test_sign_car_left_of_centre_steers_right():
  # centre > 0: lane centre is to the right of the car -> positive (right) curvature
  a = _assist(0.3)
  k = _settle(a, 10.0, _Model(centre=0.5))
  assert k > 0.0
  a = _assist(0.3)
  k = _settle(a, 10.0, _Model(centre=-0.5))
  assert k < 0.0


def test_law_magnitude_and_caps():
  v = 10.0
  d = v * lca.LANE_CENTRE_LOOKAHEAD_T
  # small offset, well inside the caps: dk = G * 2 e / d^2, no heading
  a = _assist(0.3)
  e = 0.2
  k = _settle(a, v, _Model(centre=e))
  assert math.isclose(k, 0.3 * 2 * e / d ** 2, rel_tol=1e-3)
  # heading term: centre line veering right ahead adds lambda * d * h, road curvature adds nothing
  a = _assist(0.3)
  h = 0.005
  k = _settle(a, v, _Model(centre=e, heading=h, curv=0.01))
  lam = lca.LANE_CENTRE_ZETA * math.sqrt(2 / 0.3)
  assert math.isclose(k, 0.3 * 2 * (e + lam * d * h) / d ** 2, rel_tol=1e-3)
  # large offset: capped by the curvature cap (0.4/100 = 4e-3 > 2.5e-3)
  a = _assist(0.3)
  k = _settle(a, v, _Model(centre=1.5))
  assert math.isclose(k, lca.LANE_CENTRE_K_MAX, rel_tol=1e-3)
  # at 14 m/s the lat-accel cap binds: 0.4/196 = 2.04e-3
  a = _assist(0.3)
  k = _settle(a, 14.0, _Model(centre=1.5))
  assert math.isclose(k, lca.LANE_CENTRE_LAT_ACCEL_MAX / 14.0 ** 2, rel_tol=1e-3)


def test_heading_alone_is_damping():
  # bumper centred but the centre line veers right ahead (nose pointing left): steer right
  a = _assist(0.3)
  assert _settle(a, 10.0, _Model(centre=0.0, heading=0.01)) > 0.0
  # road curvature alone (quadratic term) is not acted on -- it is already in the model command
  a = _assist(0.3)
  assert abs(_settle(a, 10.0, _Model(centre=0.0, curv=0.02))) < 1e-9


def test_gates_ramp_to_zero():
  v = 10.0
  m = _Model(centre=0.6)
  for kwargs in ({'lat_active': False}, {'lc_off': False}, {'blinker': True}):
    a = _assist(0.3)
    _settle(a, v, m)
    assert a.correction > 0.0
    k = _settle(a, v, m, **kwargs)
    assert abs(k) < 1e-6 and not a.active
  # low probability, bad width, too slow, too fast
  for model, vv in ((_Model(centre=0.6, probs=(0.3, 0.9)), v), (_Model(centre=0.6, width=5.0), v),
                    (m, 2.0), (m, 24.0)):
    a = _assist(0.3)
    assert abs(_settle(a, vv, model)) < 1e-6 and not a.active


def test_speed_fade():
  m = _Model(centre=0.4)
  # only the speed gain differs (1.0 vs 0.5), but d = v*T (clipped at D_MAX) changes too:
  # compare against the law
  def law(v, g):
    return 0.3 * g * 2 * 0.4 / min(v * lca.LANE_CENTRE_LOOKAHEAD_T, lca.LANE_CENTRE_D_MAX) ** 2
  assert math.isclose(_settle(_assist(0.3), 18.0, m), law(18.0, 1.0), rel_tol=1e-3)
  assert math.isclose(_settle(_assist(0.3), 20.5, m), law(20.5, 0.5), rel_tol=1e-3)


def test_lane_change_holdoff():
  a = _assist(0.3)
  m = _Model(centre=0.6)
  _settle(a, 10.0, m, lc_off=False)
  assert a.correction == 0.0
  # holdoff: still zero for 2 s after the lane change ends, then ramps up
  n = int(lca.LANE_CENTRE_LC_HOLDOFF_T / DT_CTRL)
  for _ in range(n - 1):
    assert a.update(True, 10.0, m, True, False) == 0.0
  assert _settle(a, 10.0, m, n=100) > 0.0


def test_slew_limits_a_lane_line_jump():
  a = _assist(0.3)
  _settle(a, 10.0, _Model(centre=0.0))
  ks = [a.update(True, 10.0, _Model(centre=1.5), True, False) for _ in range(20)]
  steps = np.diff([0.0] + ks)
  assert np.max(np.abs(steps)) <= lca.LANE_CENTRE_SLEW * DT_CTRL * 1.001


def test_closed_loop_converges_without_overshoot():
  """Kinematic bicycle at constant speed, 0.35 s actuator delay, 0.8 m initial offset
  (the model reporting the true lane centre): the assist must bring the car to the
  centre with small overshoot and no oscillation at the recommended gain."""
  delay_n = int(0.35 / DT_CTRL)
  for v in (8.0, 12.0, 18.0, 21.0):
    a = _assist(0.2)
    y = 0.8  # car 0.8 m left of centre (lane centre at +0.8 to the right)
    psi = 0.0  # heading relative to lane, positive = nose right
    q = deque([0.0] * delay_n, maxlen=delay_n)
    ys = []
    for _ in range(int(12.0 / DT_CTRL)):
      # what the model reports: centre offset at the bumper and the relative heading, straight road
      m = _Model(centre=y, heading=-psi)
      k_cmd = a.update(True, v, m, True, False)
      q.append(k_cmd)
      k = q[0]
      psi += v * k * DT_CTRL
      y -= v * math.sin(psi) * DT_CTRL
      ys.append(y)
    ys = np.array(ys)
    assert abs(ys[-1]) < 0.05, f"v={v}: did not converge ({ys[-1]:.2f})"
    assert np.min(ys) > -0.10, f"v={v}: overshoot {np.min(ys):.2f}"
    # no oscillation: after the first crossing of 0.1 m the car stays within +-0.1 m
    first = int(np.argmax(ys < 0.1))
    assert np.all(np.abs(ys[first:]) < 0.1), f"v={v}: wanders after settling"
