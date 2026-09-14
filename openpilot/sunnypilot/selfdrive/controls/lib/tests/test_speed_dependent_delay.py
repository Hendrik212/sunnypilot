"""Tests for the speed-dependent steering delay (port of sunnypilot/sunnypilot#2029)."""
import math
from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import ControlsExt


def test_highway_speed_returns_highway_delay():
  d = ControlsExt.speed_dependent_delay(30.0, 0.33, 0.30)
  assert math.isclose(d, 0.33, abs_tol=1e-9)


def test_city_speed_returns_highway_plus_boost():
  d = ControlsExt.speed_dependent_delay(10.0, 0.33, 0.30)
  assert math.isclose(d, 0.50, abs_tol=1e-9)  # 0.33 + 0.30 = 0.63, capped at 0.50


def test_capped_at_max_delay():
  d = ControlsExt.speed_dependent_delay(5.0, 0.40, 0.30)
  assert math.isclose(d, 0.50, abs_tol=1e-9)


def test_fade_is_linear_at_midpoint():
  # 50 km/h = 13.89 m/s -> city; 80 km/h = 22.22 m/s -> highway; midpoint 65 km/h
  v_mid = 65.0 / 3.6
  d = ControlsExt.speed_dependent_delay(v_mid, 0.33, 0.30)
  city = min(0.33 + 0.30, 0.50)
  assert math.isclose(d, (city + 0.33) / 2, abs_tol=1e-9)


def test_zero_boost_returns_highway_everywhere():
  for v in (5.0, 13.89, 22.22, 30.0):
    d = ControlsExt.speed_dependent_delay(v, 0.33, 0.0)
    assert math.isclose(d, 0.33, abs_tol=1e-9)


def test_highway_delay_itself_capped():
  d = ControlsExt.speed_dependent_delay(10.0, 0.60, 0.10)
  # highway capped to 0.50, city = min(0.50 + 0.10, 0.50) = 0.50
  assert math.isclose(d, 0.50, abs_tol=1e-9)


def test_monotonic_decreasing_with_speed():
  prev = ControlsExt.speed_dependent_delay(5.0, 0.33, 0.30)
  for v_kmh in range(10, 90, 5):
    v = v_kmh / 3.6
    d = ControlsExt.speed_dependent_delay(v, 0.33, 0.30)
    assert d <= prev + 1e-9, f"not monotonic at {v_kmh} km/h: {d} > {prev}"
    prev = d
