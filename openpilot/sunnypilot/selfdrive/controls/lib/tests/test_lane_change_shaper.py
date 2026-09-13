import math

import numpy as np

from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import MAX_LATERAL_JERK, clip_curvature
from openpilot.sunnypilot.selfdrive.controls.lib import lane_change_shaper as lcs
from openpilot.sunnypilot.selfdrive.controls.lib.lane_change_shaper import LaneChangeShaper, jerk_factor_for_pace


class _P:
  def __init__(self, v):
    self.v = v

  def get(self, key, return_default=False):
    assert key == "LaneChangeSmoothing"
    return self.v


def _shaper(pace):
  s = LaneChangeShaper()
  s.get_params(_P(pace))
  return s


def _run(shaper, v_ego, model_curv, in_lc):
  """Drive the shaper + clip_curvature over a model command; returns (clipped, factors)."""
  out = []
  jf = []
  prev = 0.0
  for k, lc in zip(model_curv, in_lc, strict=True):
    f = shaper.update(bool(lc), v_ego, float(k), prev)
    prev, _ = clip_curvature(v_ego, prev, float(k), 0.0, f)
    out.append(prev)
    jf.append(f)
  return np.array(out), np.array(jf)


def _lane_change_command(v_ego, t_total=4.0, width=3.5, t_pre=1.0, t_post=3.0):
  """Sinusoidal-jerk S-move: lateral accel a(t) = A sin(2 pi t / T), curvature = a / v^2."""
  n_pre = int(t_pre / DT_CTRL)
  n = int(t_total / DT_CTRL)
  n_post = int(t_post / DT_CTRL)
  t = np.arange(n) * DT_CTRL
  a_pk = 2 * math.pi * width / t_total ** 2  # full-sine accel profile for lateral displacement `width`
  a = a_pk * np.sin(2 * math.pi * t / t_total)
  k = np.concatenate([np.zeros(n_pre), a / v_ego ** 2, np.zeros(n_post)])
  in_lc = np.concatenate([np.zeros(n_pre, bool), np.ones(n, bool), np.zeros(n_post, bool)])
  return k, in_lc


def test_pace_formula_matches_starpilot():
  # StarPilot: pace 5 -> T 5.78 s, j 0.562 m/s^3, x1.3 / 5.0
  assert math.isclose(jerk_factor_for_pace(5), 0.1462, abs_tol=2e-3)
  assert jerk_factor_for_pace(10) == 1.0
  assert jerk_factor_for_pace(11) == 1.0
  assert jerk_factor_for_pace(0) == jerk_factor_for_pace(1)
  factors = [jerk_factor_for_pace(p) for p in range(1, 11)]
  assert all(a < b for a, b in zip(factors[:-1], factors[1:], strict=True))


def test_stock_pace_is_byte_for_byte_identity():
  v = 30.0
  k, in_lc = _lane_change_command(v)
  s = _shaper(10)
  out, jf = _run(s, v, k, in_lc)
  assert np.all(jf == 1.0)
  prev = 0.0
  ref = []
  for kk in k:
    prev, _ = clip_curvature(v, prev, float(kk), 0.0)
    ref.append(prev)
  assert np.array_equal(out, np.array(ref))


def test_bad_or_missing_param_is_stock():
  for v in (None, "", "abc"):
    s = LaneChangeShaper()
    s.get_params(_P(v))
    assert s.pace == 10 and s.set_jerk == 1.0


def test_entry_is_clamped_to_pace_and_arrest_is_let_through():
  v = 30.0
  k, in_lc = _lane_change_command(v)
  s = _shaper(5)
  out, jf = _run(s, v, k, in_lc)
  jerk = np.gradient(out, DT_CTRL) * v ** 2
  n_pre = int(1.0 / DT_CTRL)
  entry = slice(n_pre, n_pre + int(1.0 / DT_CTRL))
  # entry: the model asks for more than the pace clamp; the clipped command rides it
  model_jerk = np.gradient(k, DT_CTRL) * v ** 2
  assert np.max(np.abs(model_jerk[entry])) > MAX_LATERAL_JERK * s.set_jerk * 2
  assert np.max(np.abs(jerk[entry])) <= MAX_LATERAL_JERK * s.set_jerk * 1.02
  # arrest (command moving against the entry sign): allowed up to the floor, never stock
  unwinding = (np.sign(np.gradient(out)) == -1) & in_lc
  assert unwinding.any()
  assert np.max(jf[unwinding]) <= lcs.LANE_CHANGE_ARREST_JERK_FLOOR + 1e-9
  assert np.max(jf[unwinding]) > s.set_jerk * 2
  # the arrest cap is only granted while the command actually lags the model
  assert np.max(np.abs(jerk[unwinding])) <= MAX_LATERAL_JERK * lcs.LANE_CHANGE_ARREST_JERK_FLOOR * 1.02


def test_release_returns_to_stock_within_release_time():
  v = 30.0
  k, in_lc = _lane_change_command(v, t_post=4.0)
  s = _shaper(5)
  _, jf = _run(s, v, k, in_lc)
  end = int((1.0 + 4.0) / DT_CTRL)
  n_rel = int(lcs.LANE_CHANGE_SMOOTH_RELEASE_T / DT_CTRL)
  rel = jf[end:end + n_rel]
  # never below the pace clamp during the release, at least the linear schedule, and
  # stock by the end of it (the arrest pursuit may ride above the schedule while the
  # command still lags the model, which is not monotonic and is by design)
  schedule = s.set_jerk + (1.0 - s.set_jerk) * np.arange(n_rel) / n_rel
  assert np.all(rel >= s.set_jerk - 1e-12)
  assert np.all(rel >= schedule - 0.1)  # rise filter (tau 0.2 s) trails the schedule by ~0.085
  # the last of the widening is rise-filtered too (tau 0.2 s): stock within a second
  assert jf[end + n_rel + int(1.0 / DT_CTRL)] > 0.99
  assert s.entry_sign == 0.0


def test_widening_is_rise_filtered_tightening_is_immediate():
  v = 30.0
  s = _shaper(5)
  # in manoeuvre, entry latched positive, then a big negative step (arrest)
  s.update(True, v, 1e-3, 0.0)
  f1 = s.update(True, v, -5e-3, 1e-3)
  alpha = 1.0 - math.exp(-DT_CTRL / lcs.LANE_CHANGE_ARREST_RISE_TAU)
  # one step of the rise filter from set_jerk toward the arrest floor
  assert math.isclose(f1, s.set_jerk + alpha * (lcs.LANE_CHANGE_ARREST_JERK_FLOOR - s.set_jerk), rel_tol=1e-6)
  # command back in the entry direction: straight back to the tight clamp, no filter
  f2 = s.update(True, v, 2e-3, 1e-3)
  assert f2 == s.set_jerk


def test_clip_curvature_jerk_factor_only_touches_the_rate_clamp():
  v = 25.0
  # rate clamp: a big step is limited to MAX_LATERAL_JERK*jf/v^2 per frame
  full, _ = clip_curvature(v, 0.0, 1.0, 0.0, 1.0)
  half, _ = clip_curvature(v, 0.0, 1.0, 0.0, 0.5)
  assert math.isclose(half, full / 2, rel_tol=1e-9)
  # accel envelope and its saturation report are untouched
  big = 0.02
  a, lim_a = clip_curvature(v, big, big, 0.0, 1.0)
  b, lim_b = clip_curvature(v, big, big, 0.0, 0.1)
  assert a == b and lim_a == lim_b
