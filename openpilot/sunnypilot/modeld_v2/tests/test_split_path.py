import numpy as np
import pytest

from openpilot.sunnypilot.modeld_v2.split_path import DT, GATE_BP, SPEED_BP, SPLIT_HZ_MIN, SplitPath


def _signals(n=1200):
  rng = np.random.default_rng(1)
  t = np.arange(n) * DT
  a = (0.05 * np.sin(2 * np.pi * 0.3 * t) + 0.03 * np.sin(2 * np.pi * 0.8 * t + 1) + 0.02 * rng.standard_normal(n) +
       0.8 * (t > 40) * np.minimum((t - 40) / 2, 1))
  r = 0.004 * np.sin(2 * np.pi * 0.3 * t - 1) + 0.002 * np.sin(2 * np.pi * 0.8 * t) + 0.001 * rng.standard_normal(n) + 0.02
  return t, a, r, 30 + np.sin(t / 7)


def test_off_is_the_input_bit_for_bit():
  _, a, r, v = _signals()
  sp = SplitPath()
  assert all(sp.update(a[i], r[i], v[i], 0.0) == a[i] for i in range(len(a)))
  assert sp.weight == 0.0


def test_matches_the_replay_sim():
  # /mnt/sdc1/opsim/sim/splitpath.py (first-order 0.5 Hz, gate 0.3-0.6) on the same signals, 2026-11-13
  _, a, r, v = _signals()
  sp = SplitPath()
  out, react = [], []
  for i in range(len(a)):
    out.append(sp.update(a[i], r[i], v[i], 0.5))
    react.append(sp.reaction)
  assert [out[i] for i in (50, 300, 700)] == pytest.approx([0.017216619, -0.012858389, -0.027915273], abs=1e-8)
  assert [react[i] for i in (50, 300, 700)] == pytest.approx([-0.15594164, -0.048898091, -0.071690096], abs=1e-8)


def test_scene_chatter_is_filtered_the_reaction_is_not():
  n = 800
  t = np.arange(n) * DT
  scene = 0.05 * np.sin(2 * np.pi * 0.8 * t)
  sp = SplitPath()
  out = np.array([sp.update(scene[i], 0.0, 30.0, 0.5) for i in range(n)])  # car dead straight: all of it is scene
  assert out[400:].std() / scene[400:].std() == pytest.approx(0.53, abs=0.03)  # first-order 0.5 Hz at 0.8 Hz
  # the same request, but the gyro says the car wiggles: the predicted reaction passes unfiltered
  yaw = 0.004 * np.sin(2 * np.pi * 0.8 * t)
  sp = SplitPath()
  out, react = [], []
  for i in range(n):
    out.append(sp.update(0.0, yaw[i], 30.0, 0.5))
    react.append(sp.reaction)
  out, react = np.array(out[400:]), np.array(react[400:])
  assert react.std() > 0.02
  assert (out - react).std() / react.std() == pytest.approx(0.53, abs=0.03)  # what is removed is low_pass(-reaction) - (-reaction)


def test_gate_releases_a_turn_in_completely():
  _, a, r, v = _signals()
  sp = SplitPath()
  out = np.array([sp.update(a[i], r[i], v[i], 0.5) for i in range(len(a))])
  big = np.abs(a) >= GATE_BP[1]
  assert big.sum() > 100 and np.array_equal(out[big], a[big])
  assert sp.weight == 0.0


def test_highway_only():
  _, a, r, _ = _signals(400)
  sp = SplitPath()
  assert all(sp.update(a[i], r[i], SPEED_BP[0], 0.5) == a[i] for i in range(400))
  sp = SplitPath()
  out = [sp.update(a[i], r[i], SPEED_BP[1], 0.5) for i in range(400)]
  assert sp.weight == 1.0 and not np.allclose(out[100:], a[100:400])


def test_switching_on_has_no_step_and_bad_input_resets():
  _, a, r, v = _signals(400)
  sp = SplitPath()
  for i in range(200):
    sp.update(a[i], r[i], v[i], 0.0)
  before = a[199] - sp.reaction
  first = sp.update(a[200], r[200], v[200], 0.5)
  # the low-pass starts on the previous scene value: only one frame's change of the scene part can be removed
  assert abs(first - a[200]) <= abs((a[200] - sp.reaction) - before)
  assert sp.update(a[201], None, v[201], 0.5) == a[201]
  assert sp.k0 is None and sp.reaction == 0.0
  assert sp.update(a[202], float("nan"), v[202], 0.5) == a[202]


def test_cutoff_is_clamped_to_the_tested_range():
  _, a, r, v = _signals(400)
  lo, mn = SplitPath(), SplitPath()
  assert [lo.update(a[i], r[i], v[i], 0.05) for i in range(400)] == [mn.update(a[i], r[i], v[i], SPLIT_HZ_MIN) for i in range(400)]


def test_modeld_action_path_uses_it(monkeypatch):
  # the engaged path: get_action_from_model on an action-head output, with and without the split
  from types import SimpleNamespace
  from openpilot.cereal import log
  from openpilot.sunnypilot.modeld_v2 import modeld

  def run(split, hz):
    me = SimpleNamespace(CRAWL_LAT_DEADBAND=0.04, SPLIT_PATH_HZ=hz, split_path=split, LONG_SMOOTH_SECONDS=0.0, LAT_SMOOTH_SECONDS=0.0,
                         MIN_LAT_CONTROL_SPEED=0.3, generation=None)
    prev = log.ModelDataV2.Action()
    out = []
    for i in range(300):
      mo = {'action': np.array([[0.05 * np.sin(2 * np.pi * 0.8 * i * DT), 0.0]], dtype=np.float32)}
      prev = modeld.ModelState.get_action_from_model(me, mo, prev, 0.5, 0.5, 30.0, 0.0)
      out.append(prev.desiredCurvature)
    return np.array(out)

  base, off, on = run(None, 0.5), run(SplitPath(), 0.0), run(SplitPath(), 0.5)
  assert np.array_equal(base, off)
  assert on[100:].std() / base[100:].std() == pytest.approx(0.53, abs=0.04)

  monkeypatch.setattr(modeld, "_pkl_exists", lambda p: True)
  monkeypatch.setenv("COMBINED_MODEL_PKL", "/data/media/0/models/driving_cheddar_pr39255_tinygrad.pkl")
  assert modeld._split_path_model(False) and not modeld._split_path_model(True)
  monkeypatch.setenv("COMBINED_MODEL_PKL", "/data/media/0/models/driving_other_tinygrad.pkl")
  assert not modeld._split_path_model(False)
  monkeypatch.delenv("COMBINED_MODEL_PKL")
  assert not modeld._split_path_model(False)
