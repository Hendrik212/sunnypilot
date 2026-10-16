"""
Radar walls: a straight-in-the-road-frame line through the ground-stationary radar points on
each side of the car (concrete construction walls, guardrails, also a queue of stopped cars),
for dynamic lane centering's clearance guard.

Route 238 seg 14-16 (2026-10-16, construction zone): the left concrete wall fitted to
0.01-0.07 m residual from 8-29 points, 1.5-2.3 m from the car centre, while the model's road
edge sat 0.3-0.6 m beyond it and the left lane line was at 0.3-0.6 prob. On the open
highway side fits are mostly loose (0.3-0.8 m residual) and rejected by MAX_RESIDUAL.

Frame: model frame, x forward from the camera (radar dRel + RADAR_TO_CAMERA), y positive to
the RIGHT (radar yRel is positive left). The road curvature (from the model's path) is
removed before the fit and added back by wall_y(), so a wall along a bend still fits.
"""
from dataclasses import dataclass

import numpy as np

RADAR_TO_CAMERA = 1.52  # m, radard.py
MIN_V_EGO = 4.0  # m/s, below this "stationary" means nothing (radard V_EGO_STATIONARY)
STATIONARY_V = 1.5  # m/s, |vRel + vEgo| below this = ground-fixed
MIN_Y, MAX_Y = 0.8, 4.0  # m lateral window per side
MIN_X, MAX_X = 2.0, 45.0  # m
MIN_POINTS = 6
MIN_SPAN = 10.0  # m of x the points must cover
MAX_RESIDUAL = 0.10  # m RMS
OUTLIER = 0.30  # m, dropped before the refit


@dataclass
class Wall:
  y: float  # lateral at x = 0 (model frame, + right)
  slope: float
  curvature: float  # road curvature it was fitted against, 1/m
  residual: float
  points: int

  def at(self, x):
    return self.y + self.slope * x + 0.5 * self.curvature * np.square(x)


def path_curvature(model_v2) -> float:
  try:
    x = np.asarray(model_v2.position.x, dtype=float)
    y = np.asarray(model_v2.position.y, dtype=float)
    m = x < 50.0
    if m.sum() < 3:
      return 0.0
    k = 2.0 * float(np.polyfit(x[m], y[m], 2)[0])
    return k if np.isfinite(k) else 0.0
  except (AttributeError, TypeError, ValueError):
    return 0.0


def _lstsq(x: np.ndarray, yy: np.ndarray) -> tuple[np.ndarray, np.ndarray] | None:
  if len(x) < MIN_POINTS or np.ptp(x) < MIN_SPAN:
    return None
  A = np.vstack([np.ones(len(x)), x]).T
  coef = np.linalg.lstsq(A, yy, rcond=None)[0]
  return coef, yy - A @ coef


def _fit(x: np.ndarray, y: np.ndarray, k: float) -> Wall | None:
  yy = y - 0.5 * k * x ** 2
  fit = _lstsq(x, yy)
  if fit is None:
    return None
  keep = np.abs(fit[1]) < OUTLIER  # drop posts / signs, refit once
  if not keep.all():
    fit = _lstsq(x[keep], yy[keep])
    if fit is None:
      return None
  coef, res = fit
  rms = float(np.sqrt(np.mean(res ** 2)))
  if rms > MAX_RESIDUAL:
    return None
  return Wall(float(coef[0]), float(coef[1]), k, rms, len(res))


def fit_walls(points, v_ego: float, road_k: float) -> tuple[Wall | None, Wall | None]:
  """points: iterable of (dRel, yRel, vRel) in the radar frame. Returns (left, right)."""
  if v_ego < MIN_V_EGO:
    return None, None
  p = np.asarray(list(points), dtype=float).reshape(-1, 3)
  if len(p) < MIN_POINTS:
    return None, None
  x, y = p[:, 0] + RADAR_TO_CAMERA, -p[:, 1]
  ok = (np.abs(p[:, 2] + v_ego) < STATIONARY_V) & (x > MIN_X) & (x < MAX_X)
  out = []
  for sign in (-1.0, 1.0):
    m = ok & (sign * y > MIN_Y) & (sign * y < MAX_Y)
    out.append(_fit(x[m], y[m], road_k))
  return out[0], out[1]


def fill_wall(dst, wall: Wall | None) -> None:
  dst.valid = wall is not None
  if wall is not None:
    dst.y, dst.slope, dst.curvature, dst.residual, dst.points = wall.y, wall.slope, wall.curvature, wall.residual, wall.points


def read_wall(src) -> Wall | None:
  try:
    if not src.valid:
      return None
    return Wall(src.y, src.slope, src.curvature, src.residual, src.points)
  except AttributeError:
    return None


HOLD_S = 1.0  # a wall is kept this long after its last clean fit: raw fits cover 42% of the
              # route-238 zone, held 1 s 88% (2 s: 96%, but also 20% outside the zone)


class WallTracker:
  def __init__(self):
    self.walls: list[Wall | None] = [None, None]
    self._seen = [-1e9, -1e9]

  def update(self, points, v_ego: float, road_k: float, t: float) -> tuple[Wall | None, Wall | None]:
    for i, w in enumerate(fit_walls(points, v_ego, road_k)):
      if w is not None:
        self.walls[i], self._seen[i] = w, t
      elif t - self._seen[i] > HOLD_S or v_ego < MIN_V_EGO:
        self.walls[i] = None
    return self.walls[0], self.walls[1]
