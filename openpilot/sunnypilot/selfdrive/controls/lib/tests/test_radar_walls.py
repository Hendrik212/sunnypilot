import numpy as np

from openpilot.sunnypilot.selfdrive.controls.lib import radar_walls as rw

V = 18.0


def _wall_points(y_left, n=10, noise=0.03, k=0.0, seed=0):
  rng = np.random.default_rng(seed)
  d = np.linspace(4.0, 35.0, n)
  x = d + rw.RADAR_TO_CAMERA
  y_model = y_left + 0.5 * k * x ** 2 + rng.normal(0, noise, n)
  return [(di, -yi, -V) for di, yi in zip(d, y_model, strict=True)]  # radar yRel is + left


def test_fits_a_wall_and_ignores_moving_points():
  pts = _wall_points(-1.9) + [(20.0, -1.0, 0.0), (30.0, 3.0, -2.0)]  # a car ahead on the right moving with us
  left, right = rw.fit_walls(pts, V, 0.0)
  assert right is None
  assert abs(left.at(10.0) + 1.9) < 0.05 and left.residual < rw.MAX_RESIDUAL and left.points == 10


def test_bend_is_removed_before_the_fit():
  k = 4e-3
  left, _ = rw.fit_walls(_wall_points(-2.0, k=k), V, k)
  assert left is not None and abs(left.at(30.0) - (-2.0 + 0.5 * k * 900)) < 0.08


def test_scattered_points_are_not_a_wall():
  rng = np.random.default_rng(1)
  pts = [(d, -y, -V) for d, y in zip(np.linspace(4, 35, 12), rng.uniform(-3.5, -1.0, 12), strict=True)]
  assert rw.fit_walls(pts, V, 0.0) == (None, None)


def test_outlier_post_is_dropped():
  pts = _wall_points(-1.9) + [(15.0, 2.9, -V)]  # sign post 1 m further out
  left, _ = rw.fit_walls(pts, V, 0.0)
  assert left is not None and left.points == 10


def test_tracker_holds_then_drops():
  t = rw.WallTracker()
  assert t.update(_wall_points(-1.9), V, 0.0, 0.0)[0] is not None
  assert t.update([], V, 0.0, 0.9)[0] is not None
  assert t.update([], V, 0.0, 1.1)[0] is None
  assert t.update(_wall_points(-1.9), 2.0, 0.0, 1.2)[0] is None  # too slow
