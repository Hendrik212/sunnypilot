"""Lateral-acceleration deadband on an action-head model at crawl speed (ModelCrawlLatDeadband
param, m/s^2, default 0.04, 0 = off).

Action-head models (Cheddar, TSFDO) output a lateral acceleration; get_action_from_model turns
it into curvature as a / max(1, v)^2. At 0-4 km/h a bias of a few cm/s^2 becomes a curve of
30-60 m radius, and below MIN_LAT_CONTROL_SPEED that value is held while the car stands.
Routes 275-277 (2026-11-09/10, Cheddar): engaged hands-off at 0-4 km/h the raw output was
-0.017..-0.033 m/s^2 (left 80-100% of the time) while the plan path was straight (-3.6..+1.6e-3
1/m at 5 m); the low-speed angle assist of the Ioniq 6 tune mapped it to 25-55 deg of wheel and
the wheel turned left at lights. KV / CD210 (plan path) showed no such bias. Real crawl turns
ask for 0.1 m/s^2 and more (8 m radius at 1 m/s: 0.125), so a soft deadband of 0.04 removes
the bias and takes a little off those; it fades out from CRAWL_SPEED_BP[0] to [1].
"""
import numpy as np

CRAWL_DEADBAND_MAX = 0.1  # m/s^2
CRAWL_SPEED_BP = [2.0, 4.0]  # m/s: full deadband up to ~7 km/h, none from ~14 km/h
CRAWL_SPEED_V = [1.0, 0.0]


def crawl_deadband(lat_accel: float, v_ego: float, deadband: float) -> float:
  """lat_accel with |lat_accel| reduced by the speed-faded deadband (never past zero)."""
  if deadband <= 0.0:
    return lat_accel
  db = min(deadband, CRAWL_DEADBAND_MAX) * float(np.interp(v_ego, CRAWL_SPEED_BP, CRAWL_SPEED_V))
  if db <= 0.0:
    return lat_accel
  return float(np.copysign(max(abs(lat_accel) - db, 0.0), lat_accel))
