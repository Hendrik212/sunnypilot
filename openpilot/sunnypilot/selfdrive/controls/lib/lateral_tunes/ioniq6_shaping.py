"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Hyundai Ioniq 6 lateral shaping, ported from StarPilot's latcontrol_vehicle_tunes.py.

Only the 2023 firmware path is ported. StarPilot selects a separate 2025 path when the
car reports BOTH firmware strings "230915" and "240206"; ours reports only 240206, so
is_ioniq_6_2025_model() is False here and every is_ioniq_6_2025 hook is out of scope.
The caller must refuse to enable this tune on a 2025 car rather than silently applying
the 2023 shaping to it.

Extracted mechanically to avoid transcription error. StarPilot wraps every constant in
DEFAULT for live retuning; that lookup is not
ported, so those calls are folded to their defaults. Behaviour at default knobs is
identical - see tests/test_ioniq6_tune_equivalence.py, which asserts bit-equality against
the StarPilot originals over a (setpoint, jerk, v_ego) grid.

These functions are pure. Keep them that way: the equivalence test depends on it.
"""
import math
import numpy as np

from openpilot.common.constants import CV


# --- shared helpers (StarPilot: latcontrol_vehicle_tunes.py) ---

HKG_CANFD_BASE_FRICTION_THRESHOLD = 0.39

def _sigmoid(x: float) -> float:
  if x >= 0.0:
    z = math.exp(-x)
    return 1.0 / (1.0 + z)

  z = math.exp(x)
  return z / (1.0 + z)

def _gm_base_friction_threshold_default(v_ego: float) -> float:
  return float(np.interp(v_ego, [1 * CV.MPH_TO_MS, 20 * CV.MPH_TO_MS, 75 * CV.MPH_TO_MS], [0.16, 0.19, 0.27]))

def _hkg_canfd_base_friction_threshold_default(v_ego: float) -> float:
  return max(_gm_base_friction_threshold_default(v_ego), HKG_CANFD_BASE_FRICTION_THRESHOLD)

# StarPilot wraps this in an FLM runtime-override layer (_flm_base_friction_threshold) that
# lets live retuning replace the speed curve. That subsystem is not ported, so this resolves
# straight to the default path -- which is what production runs, since the override table is
# empty unless a testing-ground slot is active. See the port notes at the top of this file.
def get_hkg_canfd_base_friction_threshold(v_ego: float) -> float:
  return _hkg_canfd_base_friction_threshold_default(v_ego)


# --- Ioniq 6 constants (2023 path) ---

IONIQ_6_FF_GAIN_LEFT = 0.045
IONIQ_6_FF_GAIN_RIGHT = 0.015
IONIQ_6_BASE_LAT_ACCEL_FACTOR_MULT = 1.22
IONIQ_6_BASE_FRICTION_THRESHOLD = HKG_CANFD_BASE_FRICTION_THRESHOLD
IONIQ_6_FF_ONSET = 0.10
IONIQ_6_FF_ONSET_WIDTH = 0.04
IONIQ_6_FF_CUTOFF = 0.48
IONIQ_6_FF_CUTOFF_WIDTH = 0.12
IONIQ_6_TRANSITION_SPEED = 10.0
IONIQ_6_PHASE_SCALE = 0.10
IONIQ_6_TURN_IN_BOOST_LEFT = 1.64
IONIQ_6_TURN_IN_BOOST_RIGHT = 2.10
IONIQ_6_UNWIND_TAPER_LEFT = 3.18
IONIQ_6_UNWIND_TAPER_RIGHT = 8.20
IONIQ_6_FRICTION_MULT = 0.928
IONIQ_6_FRICTION_LAT_RISE = 0.20
IONIQ_6_FRICTION_JERK_RISE = 0.24
IONIQ_6_TURN_IN_THRESHOLD_REDUCTION_LEFT = 0.78
IONIQ_6_TURN_IN_THRESHOLD_REDUCTION_RIGHT = 1.42
IONIQ_6_UNWIND_THRESHOLD_INCREASE_LEFT = 3.90
IONIQ_6_UNWIND_THRESHOLD_INCREASE_RIGHT = 10.20
IONIQ_6_TURN_IN_FRICTION_BOOST_LEFT = 0.44
IONIQ_6_TURN_IN_FRICTION_BOOST_RIGHT = 0.94
IONIQ_6_UNWIND_FRICTION_REDUCTION_LEFT = 3.55
IONIQ_6_UNWIND_FRICTION_REDUCTION_RIGHT = 9.10
IONIQ_6_CENTER_TAPER_MAX = 0.082
IONIQ_6_CENTER_TAPER_LAT = 0.24
IONIQ_6_CENTER_TAPER_LAT_WIDTH = 0.025
IONIQ_6_CENTER_TAPER_SPEED = 18.0
IONIQ_6_CENTER_TAPER_SPEED_WIDTH = 2.5
IONIQ_6_HIGHWAY_CENTER_TAPER_MAX = 0.046
IONIQ_6_HIGHWAY_CENTER_TAPER_LAT = 0.10
IONIQ_6_HIGHWAY_CENTER_TAPER_LAT_WIDTH = 0.035
IONIQ_6_HIGHWAY_CENTER_TAPER_SPEED = 24.5
IONIQ_6_HIGHWAY_CENTER_TAPER_SPEED_WIDTH = 1.8
IONIQ_6_HIGHWAY_OUTPUT_TAPER_MAX = 0.10
IONIQ_6_HIGHWAY_OUTPUT_TAPER_LAT = 0.14
IONIQ_6_HIGHWAY_OUTPUT_TAPER_LAT_WIDTH = 0.04
IONIQ_6_HIGHWAY_OUTPUT_TAPER_SPEED = 23.5
IONIQ_6_HIGHWAY_OUTPUT_TAPER_SPEED_WIDTH = 2.0
IONIQ_6_HIGHWAY_TRANSITION_OUTPUT_TAPER_MAX = 0.18
IONIQ_6_HIGHWAY_TRANSITION_OUTPUT_TAPER_LAT = 1.05
IONIQ_6_HIGHWAY_TRANSITION_OUTPUT_TAPER_LAT_WIDTH = 0.22
IONIQ_6_HIGHWAY_TRANSITION_OUTPUT_TAPER_JERK = 0.24
IONIQ_6_HIGHWAY_TRANSITION_OUTPUT_TAPER_JERK_WIDTH = 0.14
IONIQ_6_LOW_MID_CENTER_TAPER_MAX = 0.088
IONIQ_6_LOW_MID_CENTER_TAPER_LAT = 0.28
IONIQ_6_LOW_MID_CENTER_TAPER_LAT_WIDTH = 0.06
IONIQ_6_LOW_MID_CENTER_TAPER_SPEED_MIN = 8.5
IONIQ_6_LOW_MID_CENTER_TAPER_SPEED_MAX = 16.5
IONIQ_6_LOW_MID_CENTER_TAPER_SPEED_WIDTH = 1.5
IONIQ_6_DIRECTIONAL_TAPER_LAT_START = 0.19
IONIQ_6_DIRECTIONAL_TAPER_LAT_END = 0.90
IONIQ_6_DIRECTIONAL_TAPER_LAT_WIDTH = 0.06
IONIQ_6_DIRECTIONAL_TAPER_BASE_LEFT = 0.11
IONIQ_6_DIRECTIONAL_TAPER_BASE_RIGHT = 0.45
IONIQ_6_DIRECTIONAL_TAPER_UNWIND_LEFT = 1.10
IONIQ_6_DIRECTIONAL_TAPER_UNWIND_RIGHT = 2.10
IONIQ_6_DIRECTIONAL_TAPER_FLOOR_LEFT = 0.48
IONIQ_6_DIRECTIONAL_TAPER_FLOOR_RIGHT = 0.52
IONIQ_6_DIRECTIONAL_TAPER_UNWIND_FLOOR_LEFT = 0.20
IONIQ_6_DIRECTIONAL_TAPER_UNWIND_FLOOR_RIGHT = 0.10
IONIQ_6_DIRECTIONAL_TAPER_JERK_ONSET = 1.00
IONIQ_6_DIRECTIONAL_TAPER_JERK_WIDTH = 0.30
# Unwind detection needs a softer phase transition and time smoothing than the shared
# PHASE_SCALE: desired lateral jerk noise in a sustained curve (~+/-1-2 m/s^3) otherwise
# chatters the taper between its base value and its floor at ~0.5 Hz (felt as notchy
# steering in highway sweepers).
IONIQ_6_DIRECTIONAL_TAPER_PHASE_SCALE = 0.45
IONIQ_6_DIRECTIONAL_TAPER_FILTER_RC = 0.4
IONIQ_6_DIRECTIONAL_TAPER_LOW_SPEED_RELIEF = 0.98
IONIQ_6_DIRECTIONAL_TAPER_LOW_SPEED_RELIEF_SPEED = 11.2
IONIQ_6_DIRECTIONAL_TAPER_LOW_SPEED_RELIEF_SPEED_WIDTH = 1.5
IONIQ_6_DIRECTIONAL_TAPER_LOW_SPEED_RELIEF_LAT = 0.10
IONIQ_6_DIRECTIONAL_TAPER_LOW_SPEED_RELIEF_LAT_WIDTH = 0.06
IONIQ_6_UNWIND_HIGH_SPEED_SPEED = 23.2
IONIQ_6_UNWIND_HIGH_SPEED_SPEED_WIDTH = 1.7
IONIQ_6_CRAWL_TURN_IN_FF_BOOST_LEFT = 0.18
IONIQ_6_CRAWL_TURN_IN_FF_BOOST_RIGHT = 0.24
IONIQ_6_CRAWL_TURN_IN_FF_SPEED = 5.3
IONIQ_6_CRAWL_TURN_IN_FF_SPEED_WIDTH = 1.0
IONIQ_6_CRAWL_TURN_IN_FF_LAT = 0.06
IONIQ_6_CRAWL_TURN_IN_FF_LAT_WIDTH = 0.035
IONIQ_6_LOW_SPEED_ANGLE_ASSIST_MAX_TORQUE = 0.46
IONIQ_6_LOW_SPEED_ANGLE_ASSIST_SPEED = 3.25
IONIQ_6_LOW_SPEED_ANGLE_ASSIST_SPEED_WIDTH = 0.45
IONIQ_6_LOW_SPEED_ANGLE_ASSIST_ERROR = 1.9
IONIQ_6_LOW_SPEED_ANGLE_ASSIST_ERROR_WIDTH = 1.20
IONIQ_6_LOW_SPEED_ANGLE_ASSIST_DESIRED_ANGLE = 5.5
IONIQ_6_LOW_SPEED_ANGLE_ASSIST_DESIRED_ANGLE_WIDTH = 2.4
IONIQ_6_LOW_SPEED_ANGLE_ASSIST_TRACK_RATIO_START = 0.66
IONIQ_6_LOW_SPEED_ANGLE_ASSIST_TRACK_RATIO_WIDTH = 0.12
IONIQ_6_LOW_SPEED_ANGLE_ASSIST_TRACK_RATIO_FLOOR = 0.26
IONIQ_6_LOW_SPEED_ANGLE_ASSIST_ADD_BP = [0.0, 0.35, 0.65, 1.0]
IONIQ_6_LOW_SPEED_ANGLE_ASSIST_ADD_V = [1.0, 1.0, 0.88, 0.08]
IONIQ_6_LOW_SPEED_UNWIND_ASSIST_MAX_TORQUE = 0.30
IONIQ_6_LOW_SPEED_UNWIND_ASSIST_SPEED = 3.35
IONIQ_6_LOW_SPEED_UNWIND_ASSIST_SPEED_WIDTH = 0.50
IONIQ_6_LOW_SPEED_UNWIND_ASSIST_ERROR = 1.6
IONIQ_6_LOW_SPEED_UNWIND_ASSIST_ERROR_WIDTH = 0.95
IONIQ_6_LOW_SPEED_UNWIND_ASSIST_ACTUAL_ANGLE = 10.5
IONIQ_6_LOW_SPEED_UNWIND_ASSIST_ACTUAL_ANGLE_WIDTH = 4.0
IONIQ_6_LOW_SPEED_UNWIND_ASSIST_BLEND = 0.52
IONIQ_6_HIGH_SPEED_RIGHT_TURN_IN_FF_BOOST = 0.10
IONIQ_6_HIGH_SPEED_RIGHT_TURN_IN_FF_SPEED = 18.0
IONIQ_6_HIGH_SPEED_RIGHT_TURN_IN_FF_SPEED_WIDTH = 2.5
IONIQ_6_HIGH_SPEED_RIGHT_TURN_IN_FF_LAT_START = 0.06
IONIQ_6_HIGH_SPEED_RIGHT_TURN_IN_FF_LAT_END = 0.22
IONIQ_6_HIGH_SPEED_RIGHT_TURN_IN_FF_LAT_WIDTH = 0.035
IONIQ_6_CURVY_SPEED_MIN = 7.2
IONIQ_6_CURVY_SPEED_MAX = 21.5
IONIQ_6_CURVY_SPEED_MIN_WIDTH = 1.1
IONIQ_6_CURVY_SPEED_MAX_WIDTH = 1.8
IONIQ_6_CURVY_UNWIND_EXTRA_REDUCTION_LEFT = 0.26
IONIQ_6_CURVY_UNWIND_EXTRA_REDUCTION_RIGHT = 0.30
IONIQ_6_CURVY_UNWIND_FLOOR_RELIEF_LEFT = 0.22
IONIQ_6_CURVY_UNWIND_FLOOR_RELIEF_RIGHT = 0.28
IONIQ_6_CURVY_UNWIND_LAT_START = 0.45
IONIQ_6_CURVY_UNWIND_LAT_END = 3.6
IONIQ_6_CURVY_UNWIND_LAT_ONSET_WIDTH = 0.14
IONIQ_6_CURVY_UNWIND_LAT_CUTOFF_WIDTH = 0.55
IONIQ_6_CURVY_RIGHT_UNWIND_JERK_ONSET = 0.40
IONIQ_6_CURVY_RIGHT_UNWIND_JERK_WIDTH = 0.22
IONIQ_6_CURVY_TURN_IN_TRIM_SPEED_MIN = 11.5
IONIQ_6_CURVY_TURN_IN_TRIM_SPEED_MAX = 20.5
IONIQ_6_CURVY_TURN_IN_TRIM_SPEED_WIDTH = 1.2
IONIQ_6_CURVY_TURN_IN_TRIM_LEFT = 0.08
IONIQ_6_CURVY_TURN_IN_TRIM_RIGHT = 0.09
IONIQ_6_CURVY_TURN_IN_TRIM_LAT_START = 1.0
IONIQ_6_CURVY_TURN_IN_TRIM_LAT_END = 2.5
IONIQ_6_CURVY_TURN_IN_TRIM_LAT_ONSET_WIDTH = 0.18
IONIQ_6_CURVY_TURN_IN_TRIM_LAT_CUTOFF_WIDTH = 0.30
IONIQ_6_2023_UNWIND_FF_REDUCTION_MAX = 0.18
IONIQ_6_2023_UNWIND_FF_OVERSHOOT = 0.15
IONIQ_6_2023_UNWIND_FF_OVERSHOOT_WIDTH = 0.18
IONIQ_6_2023_UNWIND_FF_JERK = 0.10
IONIQ_6_2023_UNWIND_FF_JERK_WIDTH = 0.10
IONIQ_6_2023_UNWIND_FF_SPEED_ONSET = 8.0
IONIQ_6_2023_UNWIND_FF_SPEED_ONSET_WIDTH = 2.5
IONIQ_6_2023_UNWIND_FF_SPEED_CUTOFF = 23.5
IONIQ_6_2023_UNWIND_FF_SPEED_CUTOFF_WIDTH = 2.0
IONIQ_6_LOW_SPEED_PID_RESET_SPEED = 0.1 * CV.MPH_TO_MS
# Friction compensation near zero lateral accel amplifies planner jerk noise into a slow
# (~0.5 Hz) weave on straights: the 0.09/0.39 small-signal slope plus the jerk feed acts as
# extra P/D gain right where there is no breakaway torque to overcome. Deadzone the jerk
# feed below straight-line noise levels and fade friction near center at highway speed.
IONIQ_6_FRICTION_JERK_DEADZONE = 0.30
IONIQ_6_FRICTION_CENTER_FADE_MAX = 0.50
IONIQ_6_FRICTION_CENTER_FADE_LAT = 0.15
IONIQ_6_FRICTION_CENTER_FADE_LAT_WIDTH = 0.06
IONIQ_6_FRICTION_CENTER_FADE_SPEED = 18.0
IONIQ_6_FRICTION_CENTER_FADE_SPEED_WIDTH = 2.5
IONIQ_6_HEAVY_DIRECTIONAL_TAPER_LAT_START = 0.90
IONIQ_6_HEAVY_DIRECTIONAL_TAPER_LAT_WIDTH = 0.18
IONIQ_6_HEAVY_DIRECTIONAL_TAPER_BASE_LEFT = 0.03
IONIQ_6_HEAVY_DIRECTIONAL_TAPER_BASE_RIGHT = 0.11
IONIQ_6_HEAVY_DIRECTIONAL_TAPER_UNWIND_LEFT = 0.40
IONIQ_6_HEAVY_DIRECTIONAL_TAPER_UNWIND_RIGHT = 0.55


# --- Ioniq 6 shaping functions (2023 path) ---

def _ioniq_6_sigmoid(x: float) -> float:
  return _sigmoid(x)

def _ioniq_6_low_speed_factor(v_ego: float) -> float:
  return 1.0 / (1.0 + (max(v_ego, 0.0) / IONIQ_6_TRANSITION_SPEED) ** 2)

def _ioniq_6_transition_phase(desired_lateral_accel: float, desired_lateral_jerk: float) -> float:
  return math.tanh((desired_lateral_accel * desired_lateral_jerk) / IONIQ_6_PHASE_SCALE)

def _ioniq_6_side_value(desired_lateral_accel: float, left_value: float, right_value: float) -> float:
  return left_value if desired_lateral_accel >= 0.0 else right_value

def _ioniq_6_transition_envelope(v_ego: float, desired_lateral_accel: float, desired_lateral_jerk: float) -> float:
  lat_factor = 1.0 - math.exp(-abs(desired_lateral_accel) / IONIQ_6_FRICTION_LAT_RISE)
  jerk_factor = 1.0 - math.exp(-abs(desired_lateral_jerk) / IONIQ_6_FRICTION_JERK_RISE)
  return _ioniq_6_low_speed_factor(v_ego) * lat_factor * jerk_factor

def _ioniq_6_curvy_speed_weight(v_ego: float) -> float:
  curvy_speed_min = IONIQ_6_CURVY_SPEED_MIN
  curvy_speed_max = IONIQ_6_CURVY_SPEED_MAX
  onset = _ioniq_6_sigmoid((max(v_ego, 0.0) - curvy_speed_min) / IONIQ_6_CURVY_SPEED_MIN_WIDTH)
  cutoff = _ioniq_6_sigmoid((curvy_speed_max - max(v_ego, 0.0)) / IONIQ_6_CURVY_SPEED_MAX_WIDTH)
  return onset * cutoff

def _ioniq_6_curvy_turn_in_trim_speed_weight(v_ego: float) -> float:
  curvy_turn_in_speed_min = IONIQ_6_CURVY_TURN_IN_TRIM_SPEED_MIN
  curvy_turn_in_speed_max = IONIQ_6_CURVY_TURN_IN_TRIM_SPEED_MAX
  onset = _ioniq_6_sigmoid((max(v_ego, 0.0) - curvy_turn_in_speed_min) / IONIQ_6_CURVY_TURN_IN_TRIM_SPEED_WIDTH)
  cutoff = _ioniq_6_sigmoid((curvy_turn_in_speed_max - max(v_ego, 0.0)) / IONIQ_6_CURVY_TURN_IN_TRIM_SPEED_WIDTH)
  return onset * cutoff

def get_ioniq_6_ff_scale(desired_lateral_accel: float, desired_lateral_jerk: float, v_ego: float,
                         directional_taper_scale: float | None = None) -> float:
  if desired_lateral_accel == 0.0:
    return 1.0

  gain = _ioniq_6_side_value(
    desired_lateral_accel,
    IONIQ_6_FF_GAIN_LEFT,
    IONIQ_6_FF_GAIN_RIGHT,
  )
  abs_lateral_accel = abs(desired_lateral_accel)
  onset = _ioniq_6_sigmoid((abs_lateral_accel - IONIQ_6_FF_ONSET) / IONIQ_6_FF_ONSET_WIDTH)
  cutoff = _ioniq_6_sigmoid((IONIQ_6_FF_CUTOFF - abs_lateral_accel) / IONIQ_6_FF_CUTOFF_WIDTH)
  extra_scale = gain * onset * cutoff
  phase = _ioniq_6_transition_phase(desired_lateral_accel, desired_lateral_jerk)
  turn_in_weight = max(phase, 0.0)
  unwind_weight = max(-phase, 0.0)
  low_speed_factor = _ioniq_6_low_speed_factor(v_ego)
  turn_in_boost = 1.0 + (_ioniq_6_side_value(
                          desired_lateral_accel,
                          IONIQ_6_TURN_IN_BOOST_LEFT,
                          IONIQ_6_TURN_IN_BOOST_RIGHT,
                        ) *
                          turn_in_weight * low_speed_factor)
  unwind_taper = 1.0 - (_ioniq_6_side_value(
                         desired_lateral_accel,
                         IONIQ_6_UNWIND_TAPER_LEFT,
                         IONIQ_6_UNWIND_TAPER_RIGHT,
                       ) *
                         unwind_weight * (0.30 + 0.70 * low_speed_factor))
  crawl_turn_in_scale = 0.0
  if desired_lateral_accel * desired_lateral_jerk > 0.0:
    crawl_speed_weight = _ioniq_6_sigmoid((IONIQ_6_CRAWL_TURN_IN_FF_SPEED - max(v_ego, 0.0)) /
                                          IONIQ_6_CRAWL_TURN_IN_FF_SPEED_WIDTH)
    crawl_lat_weight = _ioniq_6_sigmoid((abs_lateral_accel - IONIQ_6_CRAWL_TURN_IN_FF_LAT) /
                                        IONIQ_6_CRAWL_TURN_IN_FF_LAT_WIDTH)
    crawl_turn_in_scale = _ioniq_6_side_value(
      desired_lateral_accel,
      IONIQ_6_CRAWL_TURN_IN_FF_BOOST_LEFT,
      IONIQ_6_CRAWL_TURN_IN_FF_BOOST_RIGHT,
    ) * crawl_speed_weight * crawl_lat_weight
  high_speed_right_turn_in_scale = 0.0
  if desired_lateral_accel < 0.0 and desired_lateral_accel * desired_lateral_jerk > 0.0:
    high_speed_weight = _ioniq_6_sigmoid((max(v_ego, 0.0) - IONIQ_6_HIGH_SPEED_RIGHT_TURN_IN_FF_SPEED) /
                                         IONIQ_6_HIGH_SPEED_RIGHT_TURN_IN_FF_SPEED_WIDTH)
    high_speed_lat_onset = _ioniq_6_sigmoid((abs_lateral_accel - IONIQ_6_HIGH_SPEED_RIGHT_TURN_IN_FF_LAT_START) /
                                            IONIQ_6_HIGH_SPEED_RIGHT_TURN_IN_FF_LAT_WIDTH)
    high_speed_lat_cutoff = _ioniq_6_sigmoid((IONIQ_6_HIGH_SPEED_RIGHT_TURN_IN_FF_LAT_END - abs_lateral_accel) /
                                             IONIQ_6_HIGH_SPEED_RIGHT_TURN_IN_FF_LAT_WIDTH)
    high_speed_right_turn_in_scale = IONIQ_6_HIGH_SPEED_RIGHT_TURN_IN_FF_BOOST * high_speed_weight * high_speed_lat_onset * high_speed_lat_cutoff
  if directional_taper_scale is None:
    directional_taper_scale = get_ioniq_6_directional_taper_scale(desired_lateral_accel, desired_lateral_jerk, v_ego)
  return (1.0 + crawl_turn_in_scale + high_speed_right_turn_in_scale +
          (extra_scale * turn_in_boost * max(unwind_taper, 0.0))) * directional_taper_scale

def get_ioniq_6_2023_unwind_ff_scale(setpoint: float, measured_lateral_accel: float,
                                     desired_lateral_jerk: float, v_ego: float) -> float:
  """Trim residual curve feedforward when the 2023 car is already over-rotated."""
  if setpoint * desired_lateral_jerk >= 0.0 or setpoint * measured_lateral_accel <= 0.0:
    return 1.0

  overshoot = max(abs(measured_lateral_accel) - abs(setpoint), 0.0)
  if overshoot <= 0.0:
    return 1.0

  overshoot_weight = _ioniq_6_sigmoid((overshoot - IONIQ_6_2023_UNWIND_FF_OVERSHOOT) /
                                      IONIQ_6_2023_UNWIND_FF_OVERSHOOT_WIDTH)
  jerk_weight = _ioniq_6_sigmoid((abs(desired_lateral_jerk) - IONIQ_6_2023_UNWIND_FF_JERK) /
                                 IONIQ_6_2023_UNWIND_FF_JERK_WIDTH)
  speed_onset = _ioniq_6_sigmoid((v_ego - IONIQ_6_2023_UNWIND_FF_SPEED_ONSET) /
                                 IONIQ_6_2023_UNWIND_FF_SPEED_ONSET_WIDTH)
  speed_cutoff = _ioniq_6_sigmoid((IONIQ_6_2023_UNWIND_FF_SPEED_CUTOFF - v_ego) /
                                  IONIQ_6_2023_UNWIND_FF_SPEED_CUTOFF_WIDTH)
  reduction = (IONIQ_6_2023_UNWIND_FF_REDUCTION_MAX * overshoot_weight * jerk_weight *
               speed_onset * speed_cutoff)
  return 1.0 - reduction

def get_ioniq_6_friction_threshold(v_ego: float, desired_lateral_accel: float = 0.0, desired_lateral_jerk: float = 0.0) -> float:
  base_threshold = max(get_hkg_canfd_base_friction_threshold(v_ego), IONIQ_6_BASE_FRICTION_THRESHOLD)
  transition_envelope = _ioniq_6_transition_envelope(v_ego, desired_lateral_accel, desired_lateral_jerk)
  phase = _ioniq_6_transition_phase(desired_lateral_accel, desired_lateral_jerk)
  turn_in_weight = max(phase, 0.0)
  unwind_weight = max(-phase, 0.0)
  unwind_speed_weight = _ioniq_6_sigmoid((v_ego - IONIQ_6_UNWIND_HIGH_SPEED_SPEED) / IONIQ_6_UNWIND_HIGH_SPEED_SPEED_WIDTH)
  threshold_scale = 1.0 - (_ioniq_6_side_value(
                           desired_lateral_accel,
                           IONIQ_6_TURN_IN_THRESHOLD_REDUCTION_LEFT,
                           IONIQ_6_TURN_IN_THRESHOLD_REDUCTION_RIGHT,
                         ) *
                           transition_envelope * turn_in_weight)
  threshold_scale += (_ioniq_6_side_value(
                      desired_lateral_accel,
                      IONIQ_6_UNWIND_THRESHOLD_INCREASE_LEFT,
                      IONIQ_6_UNWIND_THRESHOLD_INCREASE_RIGHT,
                    ) *
                      transition_envelope * unwind_weight * unwind_speed_weight)
  return base_threshold * min(max(threshold_scale, 0.82), 1.18)

def get_ioniq_6_friction_scale(v_ego: float, desired_lateral_accel: float, desired_lateral_jerk: float) -> float:
  transition_envelope = _ioniq_6_transition_envelope(v_ego, desired_lateral_accel, desired_lateral_jerk)
  phase = _ioniq_6_transition_phase(desired_lateral_accel, desired_lateral_jerk)
  turn_in_weight = max(phase, 0.0)
  unwind_weight = max(-phase, 0.0)
  unwind_speed_weight = _ioniq_6_sigmoid((v_ego - IONIQ_6_UNWIND_HIGH_SPEED_SPEED) / IONIQ_6_UNWIND_HIGH_SPEED_SPEED_WIDTH)
  friction_scale = IONIQ_6_FRICTION_MULT
  friction_scale += (_ioniq_6_side_value(desired_lateral_accel, IONIQ_6_TURN_IN_FRICTION_BOOST_LEFT, IONIQ_6_TURN_IN_FRICTION_BOOST_RIGHT) *
                     transition_envelope * turn_in_weight)
  friction_scale -= (_ioniq_6_side_value(desired_lateral_accel, IONIQ_6_UNWIND_FRICTION_REDUCTION_LEFT, IONIQ_6_UNWIND_FRICTION_REDUCTION_RIGHT) *
                     transition_envelope * unwind_weight * unwind_speed_weight)
  return min(max(friction_scale, 0.82), 1.08)

def get_ioniq_6_friction_center_fade_scale(desired_lateral_accel: float, v_ego: float) -> float:
  speed_weight = _ioniq_6_sigmoid((v_ego - IONIQ_6_FRICTION_CENTER_FADE_SPEED) / IONIQ_6_FRICTION_CENTER_FADE_SPEED_WIDTH)
  center_weight = _ioniq_6_sigmoid((IONIQ_6_FRICTION_CENTER_FADE_LAT - abs(desired_lateral_accel)) / IONIQ_6_FRICTION_CENTER_FADE_LAT_WIDTH)
  return 1.0 - IONIQ_6_FRICTION_CENTER_FADE_MAX * speed_weight * center_weight

def get_ioniq_6_center_taper_scale(desired_lateral_accel: float, v_ego: float) -> float:
  speed_weight = _ioniq_6_sigmoid((v_ego - IONIQ_6_CENTER_TAPER_SPEED) / IONIQ_6_CENTER_TAPER_SPEED_WIDTH)
  center_weight = _ioniq_6_sigmoid((IONIQ_6_CENTER_TAPER_LAT - abs(desired_lateral_accel)) / IONIQ_6_CENTER_TAPER_LAT_WIDTH)
  high_speed_reduction = IONIQ_6_CENTER_TAPER_MAX * speed_weight * center_weight

  highway_speed_weight = _ioniq_6_sigmoid((v_ego - IONIQ_6_HIGHWAY_CENTER_TAPER_SPEED) / IONIQ_6_HIGHWAY_CENTER_TAPER_SPEED_WIDTH)
  highway_center_weight = _ioniq_6_sigmoid((IONIQ_6_HIGHWAY_CENTER_TAPER_LAT - abs(desired_lateral_accel)) /
                                           IONIQ_6_HIGHWAY_CENTER_TAPER_LAT_WIDTH)
  highway_center_reduction = IONIQ_6_HIGHWAY_CENTER_TAPER_MAX * highway_speed_weight * highway_center_weight

  low_mid_onset = _ioniq_6_sigmoid((v_ego - IONIQ_6_LOW_MID_CENTER_TAPER_SPEED_MIN) / IONIQ_6_LOW_MID_CENTER_TAPER_SPEED_WIDTH)
  low_mid_cutoff = _ioniq_6_sigmoid((IONIQ_6_LOW_MID_CENTER_TAPER_SPEED_MAX - v_ego) / IONIQ_6_LOW_MID_CENTER_TAPER_SPEED_WIDTH)
  low_mid_speed_weight = low_mid_onset * low_mid_cutoff
  low_mid_center_weight = _ioniq_6_sigmoid((IONIQ_6_LOW_MID_CENTER_TAPER_LAT - abs(desired_lateral_accel)) /
                                           IONIQ_6_LOW_MID_CENTER_TAPER_LAT_WIDTH)
  low_mid_reduction = IONIQ_6_LOW_MID_CENTER_TAPER_MAX * low_mid_speed_weight * low_mid_center_weight

  return 1.0 - min(high_speed_reduction + highway_center_reduction + low_mid_reduction, 0.12)

def get_ioniq_6_directional_taper_scale(desired_lateral_accel: float, desired_lateral_jerk: float, v_ego: float | None = None) -> float:
  if desired_lateral_accel == 0.0:
    return 1.0

  abs_lateral_accel = abs(desired_lateral_accel)
  onset = _ioniq_6_sigmoid((abs_lateral_accel - IONIQ_6_DIRECTIONAL_TAPER_LAT_START) / IONIQ_6_DIRECTIONAL_TAPER_LAT_WIDTH)
  cutoff = _ioniq_6_sigmoid((IONIQ_6_DIRECTIONAL_TAPER_LAT_END - abs_lateral_accel) / IONIQ_6_DIRECTIONAL_TAPER_LAT_WIDTH)
  band_weight = onset * cutoff
  heavy_band_weight = _ioniq_6_sigmoid((abs_lateral_accel - IONIQ_6_HEAVY_DIRECTIONAL_TAPER_LAT_START) / IONIQ_6_HEAVY_DIRECTIONAL_TAPER_LAT_WIDTH)
  phase = math.tanh((desired_lateral_accel * desired_lateral_jerk) / IONIQ_6_DIRECTIONAL_TAPER_PHASE_SCALE)
  unwind_weight = max(-phase, 0.0) * _ioniq_6_sigmoid((abs(desired_lateral_jerk) - IONIQ_6_DIRECTIONAL_TAPER_JERK_ONSET) /
                                                       IONIQ_6_DIRECTIONAL_TAPER_JERK_WIDTH)
  low_speed_relief_weight = 0.0
  curvy_turn_in_trim_weight = 0.0
  if v_ego is not None:
    low_speed_weight = _ioniq_6_sigmoid((IONIQ_6_DIRECTIONAL_TAPER_LOW_SPEED_RELIEF_SPEED - max(v_ego, 0.0)) /
                                        IONIQ_6_DIRECTIONAL_TAPER_LOW_SPEED_RELIEF_SPEED_WIDTH)
    tight_turn_weight = _ioniq_6_sigmoid((abs_lateral_accel - IONIQ_6_DIRECTIONAL_TAPER_LOW_SPEED_RELIEF_LAT) /
                                         IONIQ_6_DIRECTIONAL_TAPER_LOW_SPEED_RELIEF_LAT_WIDTH)
    low_speed_relief_weight = IONIQ_6_DIRECTIONAL_TAPER_LOW_SPEED_RELIEF * low_speed_weight * tight_turn_weight * (1.0 - unwind_weight)
    turn_in_weight = max(phase, 0.0)
    curvy_turn_in_speed_weight = _ioniq_6_curvy_turn_in_trim_speed_weight(v_ego)
    curvy_turn_in_lat_onset = _ioniq_6_sigmoid((abs_lateral_accel - IONIQ_6_CURVY_TURN_IN_TRIM_LAT_START) /
                                               IONIQ_6_CURVY_TURN_IN_TRIM_LAT_ONSET_WIDTH)
    curvy_turn_in_lat_cutoff = _ioniq_6_sigmoid((IONIQ_6_CURVY_TURN_IN_TRIM_LAT_END - abs_lateral_accel) /
                                                IONIQ_6_CURVY_TURN_IN_TRIM_LAT_CUTOFF_WIDTH)
    curvy_turn_in_trim_weight = curvy_turn_in_speed_weight * curvy_turn_in_lat_onset * curvy_turn_in_lat_cutoff * turn_in_weight
  base_reduction = _ioniq_6_side_value(desired_lateral_accel, IONIQ_6_DIRECTIONAL_TAPER_BASE_LEFT, IONIQ_6_DIRECTIONAL_TAPER_BASE_RIGHT)
  unwind_reduction = _ioniq_6_side_value(desired_lateral_accel, IONIQ_6_DIRECTIONAL_TAPER_UNWIND_LEFT, IONIQ_6_DIRECTIONAL_TAPER_UNWIND_RIGHT)
  heavy_base_reduction = _ioniq_6_side_value(desired_lateral_accel, IONIQ_6_HEAVY_DIRECTIONAL_TAPER_BASE_LEFT, IONIQ_6_HEAVY_DIRECTIONAL_TAPER_BASE_RIGHT)
  heavy_unwind_reduction = _ioniq_6_side_value(desired_lateral_accel, IONIQ_6_HEAVY_DIRECTIONAL_TAPER_UNWIND_LEFT, IONIQ_6_HEAVY_DIRECTIONAL_TAPER_UNWIND_RIGHT)
  base_reduction *= 1.0 - low_speed_relief_weight
  heavy_base_reduction *= 1.0 - low_speed_relief_weight
  reduction = band_weight * (base_reduction + unwind_reduction * unwind_weight)
  reduction += heavy_band_weight * (heavy_base_reduction + heavy_unwind_reduction * unwind_weight)
  reduction += (_ioniq_6_side_value(desired_lateral_accel,
                                    IONIQ_6_CURVY_TURN_IN_TRIM_LEFT,
                                    IONIQ_6_CURVY_TURN_IN_TRIM_RIGHT) *
                curvy_turn_in_trim_weight)
  curvy_unwind_weight = 0.0
  curvy_unwind_floor_relief = 0.0
  if v_ego is not None:
    curvy_unwind_phase_weight = unwind_weight
    if desired_lateral_accel < 0.0:
      curvy_unwind_phase_weight = max(-phase, 0.0) * _ioniq_6_sigmoid(
        (abs(desired_lateral_jerk) - IONIQ_6_CURVY_RIGHT_UNWIND_JERK_ONSET) / IONIQ_6_CURVY_RIGHT_UNWIND_JERK_WIDTH)
    curvy_unwind_speed_weight = _ioniq_6_curvy_speed_weight(v_ego)
    curvy_unwind_lat_onset = _ioniq_6_sigmoid((abs_lateral_accel - IONIQ_6_CURVY_UNWIND_LAT_START) /
                                              IONIQ_6_CURVY_UNWIND_LAT_ONSET_WIDTH)
    curvy_unwind_lat_cutoff = _ioniq_6_sigmoid((IONIQ_6_CURVY_UNWIND_LAT_END - abs_lateral_accel) /
                                               IONIQ_6_CURVY_UNWIND_LAT_CUTOFF_WIDTH)
    curvy_unwind_weight = curvy_unwind_speed_weight * curvy_unwind_lat_onset * curvy_unwind_lat_cutoff * curvy_unwind_phase_weight
    curvy_unwind_floor_relief = (_ioniq_6_side_value(desired_lateral_accel,
                                                     IONIQ_6_CURVY_UNWIND_FLOOR_RELIEF_LEFT,
                                                     IONIQ_6_CURVY_UNWIND_FLOOR_RELIEF_RIGHT) *
                                 curvy_unwind_weight)
  reduction += (_ioniq_6_side_value(desired_lateral_accel,
                                    IONIQ_6_CURVY_UNWIND_EXTRA_REDUCTION_LEFT,
                                    IONIQ_6_CURVY_UNWIND_EXTRA_REDUCTION_RIGHT) *
                curvy_unwind_weight)
  floor = _ioniq_6_side_value(desired_lateral_accel, IONIQ_6_DIRECTIONAL_TAPER_FLOOR_LEFT, IONIQ_6_DIRECTIONAL_TAPER_FLOOR_RIGHT)
  floor -= _ioniq_6_side_value(desired_lateral_accel, IONIQ_6_DIRECTIONAL_TAPER_UNWIND_FLOOR_LEFT, IONIQ_6_DIRECTIONAL_TAPER_UNWIND_FLOOR_RIGHT) * unwind_weight
  floor -= curvy_unwind_floor_relief
  return max(1.0 - reduction, floor)

def get_ioniq_6_highway_output_taper_scale(desired_lateral_accel: float, v_ego: float) -> float:
  speed_weight = _ioniq_6_sigmoid((v_ego - IONIQ_6_HIGHWAY_OUTPUT_TAPER_SPEED) / IONIQ_6_HIGHWAY_OUTPUT_TAPER_SPEED_WIDTH)
  center_weight = _ioniq_6_sigmoid((IONIQ_6_HIGHWAY_OUTPUT_TAPER_LAT - abs(desired_lateral_accel)) /
                                   IONIQ_6_HIGHWAY_OUTPUT_TAPER_LAT_WIDTH)
  reduction = IONIQ_6_HIGHWAY_OUTPUT_TAPER_MAX * speed_weight * center_weight
  return 1.0 - reduction

def get_ioniq_6_highway_transition_output_taper_scale(desired_lateral_accel: float, desired_lateral_jerk: float, v_ego: float) -> float:
  speed_weight = _ioniq_6_sigmoid((v_ego - IONIQ_6_HIGHWAY_OUTPUT_TAPER_SPEED) / IONIQ_6_HIGHWAY_OUTPUT_TAPER_SPEED_WIDTH)
  center_weight = _ioniq_6_sigmoid((IONIQ_6_HIGHWAY_TRANSITION_OUTPUT_TAPER_LAT - abs(desired_lateral_accel)) /
                                   IONIQ_6_HIGHWAY_TRANSITION_OUTPUT_TAPER_LAT_WIDTH)
  jerk_weight = _ioniq_6_sigmoid((abs(desired_lateral_jerk) - IONIQ_6_HIGHWAY_TRANSITION_OUTPUT_TAPER_JERK) /
                                 IONIQ_6_HIGHWAY_TRANSITION_OUTPUT_TAPER_JERK_WIDTH)
  reduction = IONIQ_6_HIGHWAY_TRANSITION_OUTPUT_TAPER_MAX * speed_weight * center_weight * jerk_weight
  return 1.0 - reduction

# --- 2023 low-speed centre error relief -------------------------------------------------
# StarPilot ships `get_ioniq_6_2025_low_speed_center_error_scale` but gates it to the 2025
# path, so the 2023 car runs the full creep gain with no near-centre relief. Measured on
# route 000001c5 seg 22 (straight stretch, 4-8 km/h, wheel swinging -29..+28 deg through
# centre): desiredLateralAccel was -0.02..-0.001 (the model wants nothing) yet the output
# slammed +1.00 -> -1.00 -> +0.71 in ~2 s. Among railed frames |P| p50 4.71 vs |ff| p50
# 0.214 -- 22x P-driven -- on a raw error of only 0.066 m/s^2. The effective creep gain
# (kp x low_speed_factor boost) is ~250 below 1.5 m/s, so 0.1 m/s^2 commands 10.7x the rail
# at 1 m/s: the loop is bang-bang for any error above ~0.01 m/s^2. The persistent error is
# road camber -- in that band |ff| p50 equals |roll_compensation| p50 exactly (30x the
# desired lat accel, from 1.5 deg of camber). Roll is the bias, P is the amplifier.
#
# Same shape as the 2025 envelope, with two deliberate differences:
#  * ERROR_SCALE 0.08, not StarPilot's 0.68, and a tighter speed gate (4.0/0.9 vs 5.0/1.3
#    -- the wider one still cut error by 6% at 8 m/s, which is normal driving). Sizing
#    against the gain table, the raw error needed to rail rises 2.0-2.9x across 3.6-11
#    km/h (e.g. 0.037 -> 0.095 at 7.2 km/h), clearing the observed 0.066 from ~7 km/h up.
#    HONEST LIMIT: below ~7 km/h it improves things 2.8x but still does not clear 0.066 --
#    the underlying gain there (effective kp ~250) is too extreme for any bounded
#    multiplier, and S=0.03 only reaches 0.062. If creep weave persists below ~7 km/h
#    after a drive, the next lever is the gain table or the roll term, not this scale.
#  * An added steering-ANGLE gate. |desiredLateralAccel| alone does NOT separate the two
#    creep regimes (at 2 m/s even full lock is only ~0.44 m/s^2), so the bare 2025 envelope
#    scores 0.507 on real parking maneuvers -- which legitimately need the rail (>=15 deg
#    steering is 40% of engaged creep frames and 85-96% of those are railed). Adding the
#    angle gate measures 0.70 on the ping-pong band and 0.000 on both parking maneuvers
#    and normal driving.
IONIQ_6_2023_LOW_SPEED_CENTER_ERROR_SCALE = 0.08
IONIQ_6_2023_LOW_SPEED_CENTER_SPEED = 4.0
IONIQ_6_2023_LOW_SPEED_CENTER_SPEED_WIDTH = 0.9
IONIQ_6_2023_LOW_SPEED_CENTER_LAT = 0.22
IONIQ_6_2023_LOW_SPEED_CENTER_LAT_WIDTH = 0.10
IONIQ_6_2023_LOW_SPEED_CENTER_JERK = 0.30
IONIQ_6_2023_LOW_SPEED_CENTER_JERK_WIDTH = 0.13
IONIQ_6_2023_LOW_SPEED_CENTER_ANGLE = 40.0
IONIQ_6_2023_LOW_SPEED_CENTER_ANGLE_WIDTH = 15.0


def _ioniq_6_2023_low_speed_center_envelope(desired_lateral_accel: float, desired_lateral_jerk: float,
                                            v_ego: float, steering_angle_deg: float) -> float:
  speed_weight = _ioniq_6_sigmoid((IONIQ_6_2023_LOW_SPEED_CENTER_SPEED - max(v_ego, 0.0)) /
                                  IONIQ_6_2023_LOW_SPEED_CENTER_SPEED_WIDTH)
  center_weight = _ioniq_6_sigmoid((IONIQ_6_2023_LOW_SPEED_CENTER_LAT - abs(desired_lateral_accel)) /
                                   IONIQ_6_2023_LOW_SPEED_CENTER_LAT_WIDTH)
  calm_weight = _ioniq_6_sigmoid((IONIQ_6_2023_LOW_SPEED_CENTER_JERK - abs(desired_lateral_jerk)) /
                                 IONIQ_6_2023_LOW_SPEED_CENTER_JERK_WIDTH)
  angle_weight = _ioniq_6_sigmoid((IONIQ_6_2023_LOW_SPEED_CENTER_ANGLE - abs(steering_angle_deg)) /
                                  IONIQ_6_2023_LOW_SPEED_CENTER_ANGLE_WIDTH)
  return speed_weight * center_weight * calm_weight * angle_weight


def get_ioniq_6_2023_low_speed_center_error_scale(desired_lateral_accel: float, desired_lateral_jerk: float,
                                                  v_ego: float, steering_angle_deg: float) -> float:
  """Cut the near-centre creep error so camber-scale error stops railing the loop.

  Returns 1.0 (no change) outside the envelope: above ~4 m/s, off centre, during a jerk
  command, or once the wheel is turned past ~40 deg.
  """
  envelope = _ioniq_6_2023_low_speed_center_envelope(desired_lateral_accel, desired_lateral_jerk,
                                                     v_ego, steering_angle_deg)
  return 1.0 - (1.0 - IONIQ_6_2023_LOW_SPEED_CENTER_ERROR_SCALE) * envelope


def get_ioniq_6_low_speed_angle_assist_torque(desired_angle_deg: float, actual_angle_deg: float,
                                              current_output_torque: float, v_ego: float) -> float:
  angle_error = desired_angle_deg - actual_angle_deg
  if desired_angle_deg * angle_error > 0.0:
    speed_weight = _ioniq_6_sigmoid((IONIQ_6_LOW_SPEED_ANGLE_ASSIST_SPEED - max(v_ego, 0.0)) /
                                    IONIQ_6_LOW_SPEED_ANGLE_ASSIST_SPEED_WIDTH)
    error_weight = _ioniq_6_sigmoid((abs(angle_error) - IONIQ_6_LOW_SPEED_ANGLE_ASSIST_ERROR) /
                                    IONIQ_6_LOW_SPEED_ANGLE_ASSIST_ERROR_WIDTH)
    desired_angle_weight = _ioniq_6_sigmoid((abs(desired_angle_deg) - IONIQ_6_LOW_SPEED_ANGLE_ASSIST_DESIRED_ANGLE) /
                                            IONIQ_6_LOW_SPEED_ANGLE_ASSIST_DESIRED_ANGLE_WIDTH)
    tracking_ratio = abs(actual_angle_deg) / max(abs(desired_angle_deg), 1e-3)
    tracking_taper = _ioniq_6_sigmoid((tracking_ratio - IONIQ_6_LOW_SPEED_ANGLE_ASSIST_TRACK_RATIO_START) /
                                      IONIQ_6_LOW_SPEED_ANGLE_ASSIST_TRACK_RATIO_WIDTH)
    tracking_scale = max(1.0 - tracking_taper, IONIQ_6_LOW_SPEED_ANGLE_ASSIST_TRACK_RATIO_FLOOR)
    assist_torque = math.copysign(
      IONIQ_6_LOW_SPEED_ANGLE_ASSIST_MAX_TORQUE *
      speed_weight * error_weight * desired_angle_weight * tracking_scale,
      -angle_error,
    )
    if abs(assist_torque) < 1e-4:
      return current_output_torque

    if current_output_torque * assist_torque >= 0.0:
      add_scale = float(np.interp(abs(current_output_torque),
                                  IONIQ_6_LOW_SPEED_ANGLE_ASSIST_ADD_BP,
                                  IONIQ_6_LOW_SPEED_ANGLE_ASSIST_ADD_V))
      return float(np.clip(current_output_torque + (assist_torque * add_scale), -1.0, 1.0))

    return float(np.clip(current_output_torque + assist_torque, -1.0, 1.0))

  speed_weight = _ioniq_6_sigmoid((IONIQ_6_LOW_SPEED_UNWIND_ASSIST_SPEED - max(v_ego, 0.0)) /
                                  IONIQ_6_LOW_SPEED_UNWIND_ASSIST_SPEED_WIDTH)
  error_weight = _ioniq_6_sigmoid((abs(angle_error) - IONIQ_6_LOW_SPEED_UNWIND_ASSIST_ERROR) /
                                  IONIQ_6_LOW_SPEED_UNWIND_ASSIST_ERROR_WIDTH)
  actual_angle_weight = _ioniq_6_sigmoid((abs(actual_angle_deg) - IONIQ_6_LOW_SPEED_UNWIND_ASSIST_ACTUAL_ANGLE) /
                                         IONIQ_6_LOW_SPEED_UNWIND_ASSIST_ACTUAL_ANGLE_WIDTH)
  assist_torque = math.copysign(IONIQ_6_LOW_SPEED_UNWIND_ASSIST_MAX_TORQUE * speed_weight * error_weight * actual_angle_weight, -angle_error)
  if abs(assist_torque) < 1e-4:
    return current_output_torque

  if current_output_torque * assist_torque >= 0.0:
    assist_torque *= IONIQ_6_LOW_SPEED_UNWIND_ASSIST_BLEND

  return float(np.clip(current_output_torque + assist_torque, -1.0, 1.0))
