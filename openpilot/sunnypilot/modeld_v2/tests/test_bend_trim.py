import numpy as np
import pytest

from openpilot.sunnypilot.modeld_v2.bend_trim import BEND_BP, DT, KP, MAX_CORRECTION, OBS_RESET, BendTrim
from openpilot.sunnypilot.modeld_v2.constants import ModelConstants

X = np.array(ModelConstants.X_IDXS)
V = 16.0


def _lines(offset, curvature, heading=0.0):
  """Lane lines as the model sees them (y right +) with the car `offset` m right of the centre of a 3.5 m lane."""
  center = -offset - heading * X + 0.5 * curvature * X**2
  return center - 1.75, center + 1.75


def _run(n, offset, lat_accel, gain, prob=0.9, v=V, active=True, camera_offset=0.0, trim=None, drift=0.0):
  trim = trim or BendTrim()
  k = lat_accel / v**2
  out = []
  for i in range(n):
    left, right = _lines(offset + drift * i * DT, k, heading=drift / v)
    out.append(trim.update(lat_accel, X, left, right, prob, prob, k * v, v, gain, active, camera_offset))
  return trim, np.array(out)


def test_off_is_the_input_bit_for_bit_but_the_observer_runs():
  trim, out = _run(200, 0.4, 2.0, 0.0)
  assert np.all(out == 2.0) and trim.correction == 0.0 and trim.weight == 0.0
  assert trim.offset == pytest.approx(0.4, abs=0.01) and abs(trim.rate) < 0.02


def test_pushes_back_toward_the_centre_in_a_bend():
  # right bend (positive), car 0.4 m to the inside (right): the request is reduced by KP * offset
  trim, out = _run(200, 0.4, 2.0, 1.0)
  assert trim.weight == pytest.approx(1.0)
  assert out[-1] - 2.0 == pytest.approx(-KP * 0.4, abs=0.02)
  # left bend, car 0.4 m to the inside (left): mirrored
  trim, out = _run(200, -0.4, -2.0, 1.0)
  assert out[-1] + 2.0 == pytest.approx(KP * 0.4, abs=0.02)
  # the gain scales it
  _, half = _run(200, 0.4, 2.0, 0.5)
  assert half[-1] - 2.0 == pytest.approx(-0.5 * KP * 0.4, abs=0.01)


def test_opposes_a_drift_to_the_inside():
  # same position, but moving to the inside at 0.3 m/s: the rate term adds to the push-back
  still, _ = _run(60, 0.2, 2.0, 1.0)
  moving, _ = _run(60, 0.2 - 0.3 * 59 * DT, 2.0, 1.0, drift=0.3)
  assert moving.rate > 0.15
  assert moving.correction < still.correction - 0.1


def test_no_correction_on_a_straight_or_without_confident_lines():
  trim, out = _run(200, 0.4, BEND_BP[0] * 0.8, 1.0)
  assert trim.weight == 0.0 and np.all(out == BEND_BP[0] * 0.8)
  trim, out = _run(200, 0.4, 2.0, 1.0, prob=0.3)
  assert trim.weight == 0.0 and np.all(out == 2.0)


def test_speed_range():
  for v in (7.0, 26.0):
    trim, out = _run(200, 0.4, 2.0, 1.0, v=v)
    assert trim.weight == 0.0 and np.all(out == 2.0)
  trim, _ = _run(200, 0.4, 2.0, 1.0, v=23.5)
  assert trim.weight == pytest.approx(0.5)


def test_fades_in_and_holds_through_the_exit():
  trim, out = _run(8, 0.4, 2.0, 1.0)
  assert 0.0 < trim.weight < 1.0 and abs(out[0] - 2.0) < 0.03  # ramp, no step
  trim, _ = _run(200, 0.4, 2.0, 1.0)
  _run(20, 0.4, 0.0, 1.0, trim=trim)  # 1 s after the bend: still on, fading
  assert 0.2 < trim.weight < 0.5
  _run(60, 0.4, 0.0, 1.0, trim=trim)
  assert trim.weight == 0.0


def test_lane_change_or_bad_input_resets():
  trim, _ = _run(200, 0.4, 2.0, 1.0)
  _, out = _run(1, 0.4, 2.0, 1.0, active=False, trim=trim)
  assert out[0] == 2.0 and trim.offset is None and trim.weight == 0.0 and trim.correction == 0.0
  left, right = _lines(0.4, 0.008)
  assert trim.update(2.0, X, left, right, 0.9, 0.9, None, V, 1.0) == 2.0
  trim.update(2.0, X, left, right, 0.9, 0.9, 0.1, V, 1.0)
  assert np.isnan(trim.update(float("nan"), X, left, right, 0.9, 0.9, 0.1, V, 1.0)) and trim.offset is None
  left[3] = np.nan
  assert trim.update(2.0, X, left, right, 0.9, 0.9, 0.1, V, 1.0) == 2.0


def test_line_jump_restarts_the_observer_and_the_correction_is_capped():
  trim, _ = _run(200, 0.1, 2.0, 1.0)
  _run(1, 0.1 + OBS_RESET + 0.2, 2.0, 1.0, trim=trim)
  assert trim.offset == pytest.approx(0.1 + OBS_RESET + 0.2, abs=0.01) and abs(trim.rate) < 0.05  # no rate spike from the jump
  _, out = _run(200, 3.0, 2.0, 1.5)
  assert out[-1] - 2.0 == pytest.approx(-MAX_CORRECTION)


def test_camera_offset_is_taken_out():
  # lane centering shifted the model's view by 0.2 m: the car is on the real centre, so nothing to correct
  trim, out = _run(200, 0.2, 2.0, 1.0, camera_offset=0.2)
  assert out[-1] == pytest.approx(2.0, abs=0.01)


def test_modeld_applies_it_on_both_request_paths():
  from types import SimpleNamespace
  from openpilot.cereal import log
  from openpilot.sunnypilot.modeld_v2 import modeld

  def run(gain, active=True):
    me = SimpleNamespace(CRAWL_LAT_DEADBAND=0.04, SPLIT_PATH_HZ=0.0, split_path=None, BEND_TRIM_GAIN=gain, bend_trim=BendTrim(), constants=ModelConstants,
                         LONG_SMOOTH_SECONDS=0.0, LAT_SMOOTH_SECONDS=0.0, MIN_LAT_CONTROL_SPEED=0.3, generation=None)
    left, right = _lines(0.4, 2.0 / V**2)
    lines = np.zeros((1, 4, len(X), 2), dtype=np.float32)
    lines[0, 1, :, 0], lines[0, 2, :, 0] = left, right
    probs = np.array([[0, 0.1, 0, 0.9, 0, 0.9, 0, 0.1]], dtype=np.float32)
    prev = log.ModelDataV2.Action()
    for _ in range(200):
      mo = {'action': np.array([[2.0, 0.0]], dtype=np.float32), 'lane_lines': lines, 'lane_lines_prob': probs}
      prev = modeld.ModelState.get_action_from_model(me, mo, prev, 0.5, 0.5, V, 2.0 / V, active)
    return prev.desiredCurvature * V**2

  assert run(0.0) == pytest.approx(2.0, abs=1e-5)
  assert run(1.0) == pytest.approx(2.0 - KP * 0.4, abs=0.02)
  assert run(1.0, active=False) == pytest.approx(2.0, abs=1e-5)
