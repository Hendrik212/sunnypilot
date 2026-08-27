"""
Regression test for get_lat_delay.

The two branches have now been inverted twice: once upstream (fixed in #1906,
53e13a7bc0) and once locally, when the 2026-08-24 merge revert (b77bcd0290)
reversed that fix back into this fork. Nothing covered it either time.

It matters beyond the toggle itself: get_lat_delay feeds controlsd_ext ->
lat_delay -> the StarPilot profile's raw_lateral_jerk denominator, so an
inverted branch silently moves every jerk-keyed constant off the operating
point its calibration assumed.
"""
from unittest.mock import MagicMock

from openpilot.sunnypilot.livedelay.helpers import get_lat_delay

STOCK = 0.4585   # a live lagd value (measured on the Ioniq 6)
CACHED = 0.25    # steerActuatorDelay + software delay, as LagdToggle caches it


def _params(toggle: bool):
  params = MagicMock()
  params.get_bool.return_value = toggle
  params.get.return_value = CACHED
  return params


def test_live_learning_on_uses_lagd_value():
  # LagdToggle on: use what lagd publishes, not the cache.
  assert get_lat_delay(_params(True), STOCK) == STOCK


def test_live_learning_off_uses_cached_value():
  # LagdToggle off: use the fixed sum LagdToggle cached.
  assert get_lat_delay(_params(False), STOCK) == CACHED


def test_branches_are_not_inverted():
  # The two branches must return different sources; an inversion swaps them.
  assert get_lat_delay(_params(True), STOCK) != get_lat_delay(_params(False), STOCK)
