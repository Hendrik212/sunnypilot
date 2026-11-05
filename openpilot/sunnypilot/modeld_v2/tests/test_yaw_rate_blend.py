import numpy as np
import pytest

from openpilot.sunnypilot.modeld_v2.constants import Plan
from openpilot.sunnypilot.modeld_v2.yaw_rate_blend import YAW_RATE_BLEND_MAX, yaw_rate_blend_correction


def _plan(r0):
  plan = np.zeros((33, 15), dtype=np.float32)
  plan[0, Plan.ORIENTATION_RATE] = [0.0, 0.0, r0]
  return plan


def test_off_by_default_and_without_a_measurement():
  plan = _plan(0.02)
  assert yaw_rate_blend_correction(plan, 27.0, 0.01, 0.0) == 0.0
  assert yaw_rate_blend_correction(plan, 27.0, None, 0.5) == 0.0
  assert yaw_rate_blend_correction(plan, 27.0, float("nan"), 0.5) == 0.0


def test_replaces_a_share_of_the_model_yaw_rate():
  # get_curvature_from_plan subtracts r0/v; the correction swaps blend * r0 for blend * measured
  plan = _plan(0.02)
  v = 27.0
  assert yaw_rate_blend_correction(plan, v, 0.01, 0.5) == pytest.approx(0.5 * (0.02 - 0.01) / v)
  assert yaw_rate_blend_correction(plan, v, 0.02, 0.5) == pytest.approx(0.0, abs=1e-9)  # plan is float32


def test_highway_only_and_capped():
  plan = _plan(0.02)
  assert yaw_rate_blend_correction(plan, 12.0, 0.0, 0.5) == 0.0  # city: untouched
  assert 0.0 < yaw_rate_blend_correction(plan, 18.0, 0.0, 0.5) < yaw_rate_blend_correction(plan, 25.0, 0.0, 0.5) * 25 / 18
  capped = yaw_rate_blend_correction(plan, 27.0, 0.0, 5.0)
  assert capped == pytest.approx(YAW_RATE_BLEND_MAX * 0.02 / 27.0)
