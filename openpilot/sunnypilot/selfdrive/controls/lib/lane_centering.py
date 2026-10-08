"""
Lane centering. Ported from StarPilot (firestar5683/StarPilot Dom 34ecbf920,
selfdrive/controls/lib/lane_centering.py): an outer position loop that trims the model's
curvature toward the centre of the two primary lane lines (plus an offset), sampled at a
speed-proportional lookahead. Gated on lane-line confidence, lane width, lane changes,
blinker and driver override; the trim is clipped and smoothed.

Changes vs. upstream, all behaviour-preserving at the defaults:
- gain, smoothing tau and centre-error deadband are constructor/set_tuning() arguments
  instead of module constants (live Params), defaults = StarPilot's values;
- the UI-only get_lane_centering_visual_direction() helper is not ported;
- update(model_updated=False) reuses the lane geometry of the last model frame. modelV2 is
  20 Hz but update() runs at 100 Hz, and re-deriving the geometry every 10 ms cost ~2% of
  core 4 on the comma 3X, enough to trip selfdrivedLagging (routes 22c/22d, 2026-10-07).
  Speed and offset are at most one model frame (50 ms) stale; smoothing stays at 100 Hz;
- update() records why it did what it did in `status` (LaneCenteringStatus) for the UI.

Frame: model y is positive to the RIGHT and positive curvature is right in this tree, so a
positive offset moves the car right of the lane centre.
"""
import math
from enum import IntEnum

from openpilot.cereal import log
import numpy as np

from openpilot.common.realtime import DT_CTRL


_MIN_V_EGO = 5.0
_MIN_LANE_PROB = 0.6
_MAX_LANE_STD = 0.3
_MIN_LANE_WIDTH = 2.6
_MAX_LANE_WIDTH = 4.8
_MAX_OFFSET = 0.3
_MIN_CENTER_TO_LINE = 1.1
_MAX_RAW_CORRECTION = 0.004
_MAX_GAIN = 0.30
_SMOOTH_TAU = 0.4
_SIGNAL_RELEASE_TAU = 0.20
_CONFIDENCE_RELEASE_TAU = 0.20
_CENTER_ERROR_DEADBAND = 0.08

_E2E_MAX_PATH_STD = 0.35
_E2E_BREAK_IN_START = 0.15
_E2E_BREAK_IN_FULL = 0.50
_E2E_MIN_LANE_AUTHORITY = 0.20
_E2E_BOUNDARY_LANE_AUTHORITY = 0.50
_E2E_BOUNDARY_MARGIN = 0.40

# below this lateral accel (|correction| * v^2) the trim counts as centred, not nudging
_NUDGE_MIN_LAT_ACCEL = 0.01  # m/s^2


def smooth_value(val: float, prev_val: float, tau: float, dt: float = DT_CTRL) -> float:
  # drive_helpers.smooth_value with math.exp: numpy scalar calls dominate this 100 Hz path on the 3X
  alpha = 1.0 - math.exp(-dt / tau) if tau > 0 else 1.0
  return alpha * val + (1.0 - alpha) * prev_val


class LaneCenteringStatus(IntEnum):
  OFF = 0          # feature disabled
  STANDBY = 1      # not engaged, too slow or model invalid
  PAUSED = 2       # blinker, driver override or lane change
  NO_LINES = 3     # lane lines not confident / implausible
  CENTERED = 4     # on target (inside the deadband or negligible trim)
  NUDGE_LEFT = 5
  NUDGE_RIGHT = 6


class LaneCenteringController:
  def __init__(self, gain: float = _MAX_GAIN, smooth_tau: float = _SMOOTH_TAU,
               deadband: float = _CENTER_ERROR_DEADBAND) -> None:
    self._correction = 0.0
    self._raw: tuple[bool, float] | None = None
    self.status = LaneCenteringStatus.OFF
    self.gain = gain
    self.smooth_tau = smooth_tau
    self.deadband = deadband

  def set_tuning(self, gain: float, smooth_tau: float, deadband: float) -> None:
    self.gain = float(np.clip(gain, 0.0, 1.0))
    self.smooth_tau = float(np.clip(smooth_tau, 0.0, 5.0))
    self.deadband = float(np.clip(deadband, 0.0, 0.5))

  @property
  def correction(self) -> float:
    return self._correction

  def reset(self) -> None:
    self._correction = 0.0
    self._raw = None

  def update(self, model_curvature, model_v2, v_ego, enabled, offset, e2e_authority, lat_active, model_valid,
             pause_on_signal=False, turn_signal_active=False, driver_override=False, model_updated=True) -> float:
    model_curvature = float(model_curvature)
    self.status = LaneCenteringStatus.STANDBY

    try:
      v_ego = float(v_ego)
      offset = float(offset)
      e2e_authority = float(e2e_authority)
    except (TypeError, ValueError):
      self.reset()
      return model_curvature

    if not (math.isfinite(v_ego) and math.isfinite(offset) and math.isfinite(e2e_authority)):
      self.reset()
      return model_curvature

    if not enabled:
      self.status = LaneCenteringStatus.OFF
      self.reset()
      return model_curvature

    if not model_valid or not lat_active or v_ego < _MIN_V_EGO:
      self.reset()
      return model_curvature

    self.status = LaneCenteringStatus.PAUSED
    if driver_override:
      self.reset()
      return model_curvature

    if pause_on_signal and turn_signal_active:
      self._raw = None
      self._correction = smooth_value(0.0, self._correction, _SIGNAL_RELEASE_TAU, dt=DT_CTRL)
      return model_curvature + self._correction

    if model_updated or self._raw is None:
      try:
        lane_change = model_v2.meta.laneChangeState != log.LaneChangeState.off
      except (AttributeError, TypeError, ValueError):
        lane_change = True
      if lane_change:
        self.reset()
        return model_curvature
      self._raw = self._raw_correction(
        model_v2,
        v_ego,
        min(max(offset, -_MAX_OFFSET), _MAX_OFFSET),
        min(max(e2e_authority, 0.0), 1.0),
        self.deadband,
      )
    valid, raw_correction = self._raw
    if not valid:
      self.status = LaneCenteringStatus.NO_LINES
      self._correction = smooth_value(0.0, self._correction, _CONFIDENCE_RELEASE_TAU, dt=DT_CTRL)
      return model_curvature + self._correction

    target = min(max(raw_correction, -_MAX_RAW_CORRECTION), _MAX_RAW_CORRECTION) * self.gain
    self._correction = smooth_value(target, self._correction, self.smooth_tau, dt=DT_CTRL)
    if abs(self._correction) * v_ego ** 2 < _NUDGE_MIN_LAT_ACCEL:
      self.status = LaneCenteringStatus.CENTERED
    else:
      self.status = LaneCenteringStatus.NUDGE_RIGHT if self._correction > 0.0 else LaneCenteringStatus.NUDGE_LEFT
    return model_curvature + self._correction

  @staticmethod
  def _valid_path(x, y) -> bool:
    return x.size >= 2 and x.size == y.size and np.isfinite(x).all() and np.isfinite(y).all() and np.all(np.diff(x) > 0)

  @staticmethod
  def _covers(x, distance: float) -> bool:
    return bool(x[0] <= distance <= x[-1])

  @staticmethod
  def _raw_correction(model_v2, v_ego: float, offset: float, e2e_authority: float,
                      deadband: float = _CENTER_ERROR_DEADBAND) -> tuple[bool, float]:
    try:
      lane_lines = model_v2.laneLines
      probs = np.asarray(model_v2.laneLineProbs, dtype=float)
      stds = np.asarray(model_v2.laneLineStds, dtype=float)
      if len(lane_lines) < 3 or probs.size < 3 or stds.size < 3:
        return False, 0.0
      if not np.isfinite(probs[[1, 2]]).all() or not np.isfinite(stds[[1, 2]]).all():
        return False, 0.0
      if np.any(probs[[1, 2]] < _MIN_LANE_PROB) or np.any(probs[[1, 2]] > 1.0):
        return False, 0.0
      if np.any(stds[[1, 2]] < 0.0) or np.any(stds[[1, 2]] > _MAX_LANE_STD):
        return False, 0.0

      left_x = np.asarray(lane_lines[1].x, dtype=float)
      left_y = np.asarray(lane_lines[1].y, dtype=float)
      right_x = np.asarray(lane_lines[2].x, dtype=float)
      right_y = np.asarray(lane_lines[2].y, dtype=float)
      pos_x = np.asarray(model_v2.position.x, dtype=float)
      pos_y = np.asarray(model_v2.position.y, dtype=float)
      if not (LaneCenteringController._valid_path(left_x, left_y) and
              LaneCenteringController._valid_path(right_x, right_y) and
              LaneCenteringController._valid_path(pos_x, pos_y)):
        return False, 0.0

      lookahead = float(np.clip(v_ego, 8.0, 35.0))
      if not all(LaneCenteringController._covers(x, lookahead) for x in (left_x, right_x, pos_x)):
        return False, 0.0

      left = float(np.interp(lookahead, left_x, left_y))
      right = float(np.interp(lookahead, right_x, right_y))
      width = right - left
      if not _MIN_LANE_WIDTH <= width <= _MAX_LANE_WIDTH:
        return False, 0.0

      max_safe_offset = min(_MAX_OFFSET, max(0.0, width * 0.5 - _MIN_CENTER_TO_LINE))
      target_y = 0.5 * (left + right) + float(np.clip(offset, -max_safe_offset, max_safe_offset))
      model_y = float(np.interp(lookahead, pos_x, pos_y))
      error = target_y - model_y
      error_abs = abs(error)
      if error_abs <= deadband:
        error = 0.0
      else:
        error = np.copysign(error_abs - deadband, error)

      try:
        pos_y_std = np.asarray(model_v2.position.yStd, dtype=float)
        if LaneCenteringController._valid_path(pos_x, pos_y_std):
          path_std = float(np.interp(lookahead, pos_x, pos_y_std))
          if 0.0 <= path_std <= _E2E_MAX_PATH_STD:
            break_in = np.clip(
              (error_abs - _E2E_BREAK_IN_START) / (_E2E_BREAK_IN_FULL - _E2E_BREAK_IN_START),
              0.0,
              1.0,
            )
            path_clearance = min(model_y - left, right - model_y)
            boundary_weight = float(np.clip(
              (_MIN_CENTER_TO_LINE + _E2E_BOUNDARY_MARGIN - path_clearance) / _E2E_BOUNDARY_MARGIN,
              0.0,
              1.0,
            ))
            lane_authority = _E2E_MIN_LANE_AUTHORITY + boundary_weight * (
              _E2E_BOUNDARY_LANE_AUTHORITY - _E2E_MIN_LANE_AUTHORITY
            )
            error *= 1.0 - e2e_authority * float(break_in) * (1.0 - lane_authority)
      except (AttributeError, TypeError, ValueError):
        pass

      return True, float(2.0 * error / lookahead ** 2)
    except (AttributeError, IndexError, TypeError, ValueError):
      return False, 0.0


def get_raw_lane_centering_correction(model_v2, v_ego: float, offset: float, e2e_authority: float,
                                      deadband: float = _CENTER_ERROR_DEADBAND) -> tuple[bool, float]:
  """Return the instantaneous lane-centering correction without controller filtering."""
  return LaneCenteringController._raw_correction(model_v2, v_ego, offset, e2e_authority, deadband)
