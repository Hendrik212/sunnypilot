"""Tests for the low-speed lateral accel cap (MaxLateralAccel)."""
import math
from openpilot.selfdrive.controls.lib.drive_helpers import MAX_LATERAL_ACCEL_NO_ROLL, clip_curvature
from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import ControlsExt


def test_full_value_below_60_kmh():
  for kph in (10.0, 40.0, 60.0):
    assert math.isclose(ControlsExt.speed_dependent_lat_accel(kph / 3.6, 4.5), 4.5, abs_tol=1e-9)


def test_stock_at_and_above_80_kmh():
  for kph in (80.0, 130.0):
    assert math.isclose(ControlsExt.speed_dependent_lat_accel(kph / 3.6, 4.5), MAX_LATERAL_ACCEL_NO_ROLL, abs_tol=1e-9)


def test_fade_is_linear_at_midpoint():
  a = ControlsExt.speed_dependent_lat_accel(70.0 / 3.6, 4.0)
  assert math.isclose(a, 3.5, abs_tol=1e-9)


def test_stock_value_is_a_no_op():
  for kph in (10.0, 70.0, 130.0):
    a = ControlsExt.speed_dependent_lat_accel(kph / 3.6, MAX_LATERAL_ACCEL_NO_ROLL)
    assert math.isclose(a, MAX_LATERAL_ACCEL_NO_ROLL, abs_tol=1e-9)


def test_clip_curvature_uses_the_raised_cap():
  v = 10.0
  want = 4.0 / v ** 2  # model asks 4 m/s^2; start there so the jerk clamp is not involved
  stock, lim_stock = clip_curvature(v, want, want, 0.0)
  raised, lim_raised = clip_curvature(v, want, want, 0.0, 1.0, 4.5)
  assert math.isclose(stock * v ** 2, 3.0, abs_tol=1e-9) and lim_stock
  assert math.isclose(raised, want, abs_tol=1e-12) and not lim_raised
