"""Measured yaw rate in the plan-to-curvature step (ModelYawRateBlend param, default 0 = off).

get_curvature_from_plan returns 2*psi(t)/(v*t) - r0/v, where r0 is the plan's yaw rate at t=0: the
model's estimate of the car's CURRENT yaw rate, inferred from the camera frames, so it carries the
model's perception lag and image noise. deviceMotion measures the same quantity from the gyro
(~45 ms behind the raw gyro at 20 Hz, route 0000026a). Blending it in shortens the outer lane loop
(model -> car -> camera) where it is shortest of phase.

Replay sim, route 0000026a segs 14-16 (real video + CD210 + the car's controller + fitted EPS):
the outer loop crosses over at 0.30 Hz with 13-21 deg phase margin. Full substitution (blend 1.0)
closes a fast inner loop through the EPS and goes unstable; a 0.5 blend stayed stable with weave
-7..-10% alone and -26..-27% together with RippleNotchQ 4 (deviceMotion latency), chatter x1.6-1.9.
Highway only: the gyro term is divided by speed, and the weave is a highway problem.
"""
import numpy as np

from openpilot.sunnypilot.modeld_v2.constants import Plan

YAW_RATE_BLEND_MAX = 0.7
YAW_RATE_BLEND_SPEED_BP = [16.0, 20.0]  # m/s: off below ~58 km/h, full from 72 km/h
YAW_RATE_BLEND_SPEED_V = [0.0, 1.0]
MIN_SPEED = 1.0


def yaw_rate_blend_correction(plan: np.ndarray, v_ego: float, yaw_rate_measured: float | None, blend: float) -> float:
  """Curvature to add to get_curvature_from_plan's output: blend * (r0_model - r_measured) / v.
  Same sign convention as desiredCurvature and deviceMotion.angularVelocityDevice.z."""
  if blend <= 0.0 or yaw_rate_measured is None or not np.isfinite(yaw_rate_measured):
    return 0.0
  w = float(np.interp(v_ego, YAW_RATE_BLEND_SPEED_BP, YAW_RATE_BLEND_SPEED_V))
  if w <= 0.0:
    return 0.0
  r0_model = float(plan[0, Plan.ORIENTATION_RATE][2])
  return min(blend, YAW_RATE_BLEND_MAX) * w * (r0_model - yaw_rate_measured) / max(v_ego, MIN_SPEED)
