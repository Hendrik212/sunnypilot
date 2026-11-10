import pytest

from openpilot.sunnypilot.modeld_v2.crawl_deadband import CRAWL_DEADBAND_MAX, CRAWL_SPEED_BP, crawl_deadband


def test_removes_the_crawl_bias():
  # routes 275-277: -0.017..-0.033 m/s^2 at 0-4 km/h with a straight plan
  for a in (-0.033, -0.017, 0.02):
    assert crawl_deadband(a, 0.5, 0.04) == 0.0


def test_real_crawl_turns_pass_minus_the_deadband():
  assert crawl_deadband(0.125, 1.0, 0.04) == pytest.approx(0.085)
  assert crawl_deadband(-0.5, 1.5, 0.04) == pytest.approx(-0.46)


def test_fades_out_with_speed():
  assert crawl_deadband(0.03, CRAWL_SPEED_BP[1], 0.04) == 0.03
  assert crawl_deadband(0.03, 20.0, 0.04) == 0.03
  mid = crawl_deadband(0.05, 0.5 * sum(CRAWL_SPEED_BP), 0.04)
  assert 0.01 < mid < 0.05


def test_off_and_capped():
  assert crawl_deadband(-0.03, 0.5, 0.0) == -0.03
  assert crawl_deadband(0.5, 0.5, 1.0) == pytest.approx(0.5 - CRAWL_DEADBAND_MAX)
