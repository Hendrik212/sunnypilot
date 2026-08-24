"""
StarPilot Ioniq 6 lateral tune profile.

Ported from StarPilot's selfdrive/controls/lib/latcontrol_torque.py (2023 firmware path
only; the 2025 path is deliberately not ported -- see latcontrol_ioniq6_tune.py). The pure
shaping math and its constants live in latcontrol_ioniq6_tune.py; this module owns the
mutable controller state and the inner loop that calls into it.

Runs in lateral-acceleration space, same as the upstream v2 path, so the multiplicative ff
and output_torque scales convert cleanly to torque at the end. LatControlTorqueExt is a
passthrough here (LateralJerkTorqueController is gated off for this tune), so the shaping
done here is what reaches the actuator.
"""
import math
from collections import deque

import numpy as np

from opendbc.car.lateral import get_friction
from opendbc.sunnypilot.car.hyundai.values import IONIQ6_STARPILOT_TORQUE
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.sunnypilot.selfdrive.controls.lib import latcontrol_ioniq6_tune as i6
from openpilot.sunnypilot.selfdrive.controls.lib.lateral_tunes.base import LateralTuneProfile

# --- machinery this tune depends on that upstream v2 does not have ---
# Ported verbatim from StarPilot's latcontrol_torque.py. None of these are referenced by
# the upstream control path.
MAX_LAT_JERK_UP = 2.5             # m/s^3
JERK_GAIN = 0.22
LP_FILTER_CUTOFF_HZ = 1.2
MIN_LATERAL_CONTROL_SPEED = 0.3   # m/s
STEER_RELEASE_I_DECAY = 0.8
UNWIND_D_DES_THRESHOLD = -1.0     # m/s^3
UNWIND_LAT_ACCEL_NEAR_ZERO = 0.3  # m/s^2
FF_ROLL_OFFSET_FADE_BP = [0.5, 2.5]  # m/s
FF_ROLL_OFFSET_FADE_V = [0.0, 1.0]
LOW_SPEED_X = [0, 10, 20, 30]
LOW_SPEED_Y = [12, 10.5, 8, 5]

# Small planner jerk changes around the lane center can repeatedly re-trigger the friction
# compensation term. Keep this correction out of the center band while leaving actual
# turn-in and unwind commands unchanged.
CENTER_CHATTER_JERK_DEADZONE_SPEED_BP = [0.0, 5.0, 12.0, 25.0]  # m/s
CENTER_CHATTER_JERK_DEADZONE_SPEED_V = [0.08, 0.12, 0.18, 0.18]  # m/s^3
CENTER_CHATTER_JERK_DEADZONE_LAT_ACCEL_BP = [0.0, 0.18, 0.35]   # m/s^2
CENTER_CHATTER_JERK_DEADZONE_LAT_ACCEL_V = [1.0, 1.0, 0.0]

# StarPilot's modeld sets LAT_SMOOTH_SECONDS = 0.1; the shared modeld here sets 0.0, and
# controlsd passes lat_delay = lateralDelay + LAT_SMOOTH_SECONDS. Adding the difference
# back inside the profile therefore reproduces StarPilot's lat_delay EXACTLY, not
# approximately. It matters: lat_delay is the denominator of raw_lateral_jerk, so without
# it every jerk value is ~22% larger than the scale these constants were calibrated on
# (measured lateralDelay on this car is 0.4585 s: 0.4585 vs 0.5585). The jerk-keyed
# constants that would otherwise sit off their operating point are
# IONIQ_6_DIRECTIONAL_TAPER_JERK_ONSET, IONIQ_6_FRICTION_JERK_RISE,
# IONIQ_6_FRICTION_JERK_DEADZONE, IONIQ_6_2023_UNWIND_FF_JERK,
# IONIQ_6_HIGHWAY_TRANSITION_OUTPUT_TAPER_JERK and the MAX_LAT_JERK_UP clip.
#
# Only the controller-side effect is reproduced. StarPilot's 0.1 also low-passes the model's
# published desiredCurvature and shifts the model's own action horizon (lat_action_t); both
# live in modeld, and neither is reproduced here. See the profile docs for why.
LAT_SMOOTH_SECONDS_OFFSET = 0.1


def get_center_chatter_friction_jerk_deadzone(v_ego, setpoint, vehicle_deadzone=0.0):
  """Return the small-signal jerk deadzone without changing turn commands."""
  speed_deadzone = np.interp(max(v_ego, 0.0), CENTER_CHATTER_JERK_DEADZONE_SPEED_BP,
                             CENTER_CHATTER_JERK_DEADZONE_SPEED_V)
  center_weight = np.interp(abs(setpoint), CENTER_CHATTER_JERK_DEADZONE_LAT_ACCEL_BP,
                            CENTER_CHATTER_JERK_DEADZONE_LAT_ACCEL_V)
  return max(float(vehicle_deadzone), float(speed_deadzone * center_weight))


class Ioniq6StarPilotProfile(LateralTuneProfile):
  profile_id = "ioniq6_starpilot"

  lat_delay_offset = 0.0  # see LAT_SMOOTH_SECONDS_OFFSET

  # StarPilot uses drive_helpers.MIN_SPEED (1.0) as the low-speed-factor denominator floor.
  # A lower floor makes the boost blow up at creep: at 0.3 m/s a floor of 0.3 gives
  # (12/0.3)^2 = 1600 against StarPilot's (12/1.0)^2 = 144, i.e. an effective proportional
  # gain 3.6x StarPilot's in exactly the 0.3-1.0 m/s band where lateral is live in
  # stop-and-go.
  low_speed_factor_min_speed = MIN_LATERAL_CONTROL_SPEED

  # This tune ships a fixed torque baseline (3.0 / 0.09). torqued seeds its filter from
  # CP.lateralTuning.torque -- the car's override.toml entry, [2.5, 2.5, 0.005] -- and
  # controlsd feeds that back over the baseline on every frame that useParams is set, so
  # allowing live params here silently replaces the tune. Measured on-device after a full
  # hour of driving: valid=False at calPerc=55, latAccelFactorFiltered=2.5,
  # frictionCoefficientFiltered=0.005, i.e. torqued had published nothing but the seed
  # while its own raw estimate read 4.0 / 0.058. With friction at 0.005 the feedforward is
  # ~5% of intended, and at creep the friction term IS the entire feedforward.
  use_live_torque_params = True

  def init_controller(self, ctl, CP, CP_SP, CI) -> None:
    # The controller owns the StarPilot baseline rather than inheriting it from CP. CP is
    # written once at fingerprint time, so on a live switch from upstream it still holds the
    # UPSTREAM factor -- multiplying that by 1.22 would give a gain that is neither tune.
    # Setting the baseline here makes the switch stateless.
    #
    # The 1.22 multiplier is the dominant gain term and easy to miss: StarPilot's effective
    # latAccelFactor is 3.0 x 1.22 = 3.66, giving 409/3.66 = 112 CAN per m/s^2 vs a measured
    # plant-neutral ~105. Without it the tune lands 22% hot.
    ctl.torque_params.latAccelFactor = (IONIQ6_STARPILOT_TORQUE['LAT_ACCEL_FACTOR'] *
                                        i6.IONIQ_6_BASE_LAT_ACCEL_FACTOR_MULT)
    ctl.torque_params.friction = IONIQ6_STARPILOT_TORQUE['FRICTION']
    ctl.update_limits()

    # StarPilot stores curvature in the request buffer (scales by v^2 on read); upstream v2
    # stored lateral accel directly. Storing lateral accel makes the delayed request lag the
    # measurement whenever speed is changing, which at creep-speed gains reads as a phantom
    # unwind error during pull-away.
    self.curvature_request_buffer = deque([0.] * ctl.lat_accel_request_buffer_len,
                                          maxlen=ctl.lat_accel_request_buffer_len)
    self.jerk_filter = FirstOrderFilter(0.0, 1 / (2 * np.pi * LP_FILTER_CUTOFF_HZ), ctl.dt)
    self.directional_taper_filter = FirstOrderFilter(1.0, i6.IONIQ_6_DIRECTIONAL_TAPER_FILTER_RC, ctl.dt)
    self.prev_steering_pressed = False
    self.prev_desired_lateral_accel = 0.0

    ctl.measurement_rate_filter = FirstOrderFilter(0.0, 1 / (2 * np.pi * (MAX_LAT_JERK_UP - 0.5)), ctl.dt)
    ctl.low_speed_reset_threshold = min(max(CP.minSteerSpeed, MIN_LATERAL_CONTROL_SPEED),
                                        i6.IONIQ_6_LOW_SPEED_PID_RESET_SPEED)

  def apply_live_torque_params(self, ctl, latAccelFactor, latAccelOffset, friction) -> None:
    """Not reached while use_live_torque_params is False; kept so the 1.22 normalization
    stays with the profile if live learning is ever re-enabled for this tune. StarPilot
    applies its multiplier here too, so torqued learns the UNMULTIPLIED factor and the
    controller multiplies on use, keeping the sanity window and the feedforward on one
    consistent normalization."""
    ctl.torque_params.latAccelFactor = latAccelFactor * i6.IONIQ_6_BASE_LAT_ACCEL_FACTOR_MULT
    ctl.torque_params.latAccelOffset = latAccelOffset
    ctl.torque_params.friction = friction

  def prime_inactive(self, ctl, CS, desired_curvature, measurement) -> None:
    # Keep the request buffer, the rate state and the directional taper primed with the live
    # command (which tracks the measured curvature while inactive) instead of zeroing them.
    # Re-engaging with a wound wheel against a zeroed buffer puts the setpoint ~lat_delay
    # behind the measurement, and the low-speed gains turn that lag into a hard unwind shove.
    self.curvature_request_buffer.append(desired_curvature)
    self.jerk_filter.x = 0.0
    self.prev_desired_lateral_accel = desired_curvature * CS.vEgo ** 2
    self.directional_taper_filter.x = 1.0
    # StarPilot primes previous_measurement with the live measurement, not zero, and resets
    # the PID every inactive frame so a re-engage never starts from a wound integrator.
    ctl.previous_measurement = 0.0
    ctl.measurement_rate_filter.x = 0.0

  def update(self, ctl, active, CS, VM, params, steer_limited_by_safety,
             desired_curvature, measured_curvature, measurement, calibrated_pose,
             pid_log, lat_delay) -> float:
    # Reproduce StarPilot's lat_delay (see LAT_SMOOTH_SECONDS_OFFSET).
    lat_delay = lat_delay + self.lat_delay_offset

    # steering-release I decay: bleed integrator when the driver lets go, so a resumed
    # hold does not start from a wound-up state.
    if self.prev_steering_pressed and not CS.steeringPressed:
      ctl.pid.i *= STEER_RELEASE_I_DECAY
    self.prev_steering_pressed = CS.steeringPressed

    # Roll compensation fades in below walking pace; an unfaded road-crown term
    # dominates the whole feedforward at pull-away and actively unwinds a held wheel.
    roll_offset_fade = np.interp(CS.vEgo, FF_ROLL_OFFSET_FADE_BP, FF_ROLL_OFFSET_FADE_V)
    roll_compensation = params.roll * ACCELERATION_DUE_TO_GRAVITY * roll_offset_fade
    curvature_deadzone = abs(VM.calc_curvature(math.radians(ctl.steering_angle_deadzone_deg), CS.vEgo, 0.0))
    lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2

    # Curvature-space request buffer (StarPilot): scales by v^2 on read so the delayed
    # request does not lag the measurement when speed is changing.
    delay_frames = int(np.clip(lat_delay / ctl.dt, 1, ctl.lat_accel_request_buffer_len))
    expected_lateral_accel = self.curvature_request_buffer[-delay_frames] * CS.vEgo ** 2
    self.curvature_request_buffer.append(desired_curvature)
    future_desired_lateral_accel = desired_curvature * CS.vEgo ** 2

    raw_lateral_jerk = (future_desired_lateral_accel - expected_lateral_accel) / max(lat_delay, ctl.dt)
    raw_lateral_jerk = np.clip(raw_lateral_jerk, -MAX_LAT_JERK_UP, MAX_LAT_JERK_UP)
    desired_lateral_jerk = np.clip(self.jerk_filter.update(raw_lateral_jerk), -MAX_LAT_JERK_UP, MAX_LAT_JERK_UP)

    gravity_adjusted_future_lateral_accel = future_desired_lateral_accel - roll_compensation
    setpoint = expected_lateral_accel + desired_lateral_jerk * lat_delay
    desired_lateral_accel_rate = (setpoint - self.prev_desired_lateral_accel) / ctl.dt
    # Unwind detection: when the desired accel is near zero and decreasing, the car is
    # coming out of a curve. Freezing the integrator here prevents windup against the
    # soon-to-be-reduced command -- the core of the "slow curve unwind" complaint.
    unwind_detected = (desired_lateral_accel_rate < UNWIND_D_DES_THRESHOLD and
                       abs(setpoint) < UNWIND_LAT_ACCEL_NEAR_ZERO)
    self.prev_desired_lateral_accel = setpoint

    measurement_rate = ctl.measurement_rate_filter.update((measurement - ctl.previous_measurement) / ctl.dt)
    measurement_rate = np.clip(measurement_rate, -MAX_LAT_JERK_UP, MAX_LAT_JERK_UP)
    ctl.previous_measurement = measurement

    # Low-speed factor boosts the proportional gain at creep; applied to the error so
    # the friction term (which feeds on error) sees it too.
    low_speed_factor = (np.interp(CS.vEgo, LOW_SPEED_X, LOW_SPEED_Y) /
                        max(CS.vEgo, self.low_speed_factor_min_speed)) ** 2
    current_kp = np.interp(CS.vEgo, ctl.pid._k_p[0], ctl.pid._k_p[1])
    error = setpoint - measurement
    error_with_lsf = error * (1 + low_speed_factor / max(current_kp, 1e-3))

    pid_log.error = float(error_with_lsf)
    pid_log.actualLateralAccel = float(measurement)
    pid_log.desiredLateralAccel = float(setpoint)
    pid_log.desiredLateralJerk = float(desired_lateral_jerk)

    # --- feedforward ---
    ff = gravity_adjusted_future_lateral_accel
    ff -= ctl.torque_params.latAccelOffset * roll_offset_fade

    center_taper = i6.get_ioniq_6_center_taper_scale(setpoint, CS.vEgo)
    # Smooth the directional taper so jerk-gated unwind cuts can't step the FF in one frame.
    directional_taper = self.directional_taper_filter.update(
      i6.get_ioniq_6_directional_taper_scale(setpoint, desired_lateral_jerk, CS.vEgo))
    ff *= i6.get_ioniq_6_ff_scale(setpoint, desired_lateral_jerk, CS.vEgo,
                                  directional_taper_scale=directional_taper) * center_taper
    # 2023 path only: the 2025 unwind scale is gated out (see is_ioniq_6_2025_model).
    ff *= i6.get_ioniq_6_2023_unwind_ff_scale(setpoint, measurement, desired_lateral_jerk, CS.vEgo)

    # --- friction ---
    friction_threshold = i6.get_ioniq_6_friction_threshold(CS.vEgo, setpoint, desired_lateral_jerk) / max(center_taper, 1e-3)
    friction_scale = i6.get_ioniq_6_friction_scale(CS.vEgo, setpoint, desired_lateral_jerk)
    friction_scale = 1.0 + ((friction_scale - 1.0) * center_taper)
    friction_scale *= i6.get_ioniq_6_friction_center_fade_scale(setpoint, CS.vEgo)
    friction_jerk_deadzone = get_center_chatter_friction_jerk_deadzone(
      CS.vEgo, setpoint, i6.IONIQ_6_FRICTION_JERK_DEADZONE)
    friction_jerk = math.copysign(max(abs(desired_lateral_jerk) - friction_jerk_deadzone, 0.0),
                                  desired_lateral_jerk)
    ff += friction_scale * get_friction(error_with_lsf + JERK_GAIN * friction_jerk,
                                        lateral_accel_deadzone, friction_threshold, ctl.torque_params)

    # --- PID + output ---
    if CS.vEgo < ctl.low_speed_reset_threshold:
      ctl.pid.reset()
    freeze_integrator = (steer_limited_by_safety or CS.steeringPressed or
                         CS.vEgo < ctl.low_speed_reset_threshold or unwind_detected)
    output_lataccel = ctl.pid.update(pid_log.error, error_rate=-measurement_rate,
                                     feedforward=ff, speed=CS.vEgo,
                                     freeze_integrator=freeze_integrator)
    output_torque = ctl.torque_from_lateral_accel(output_lataccel, ctl.torque_params)

    # Low-speed angle assist: below ~3.25 m/s the curvature-to-torque relationship is
    # poor, so blend in a direct angle-error torque. Not applied when the driver is
    # steering (the driver owns the wheel then).
    if not CS.steeringPressed:
      desired_angle_no_offset = math.degrees(VM.get_steer_from_curvature(-desired_curvature, CS.vEgo, params.roll))
      actual_angle_no_offset = CS.steeringAngleDeg - params.angleOffsetDeg
      output_torque = i6.get_ioniq_6_low_speed_angle_assist_torque(desired_angle_no_offset, actual_angle_no_offset,
                                                                   output_torque, CS.vEgo)

    # Highway output tapers: trim authority at high speed / during transitions to keep
    # sweepers smooth rather than notchy.
    output_torque *= i6.get_ioniq_6_highway_output_taper_scale(setpoint, CS.vEgo)
    output_torque *= i6.get_ioniq_6_highway_transition_output_taper_scale(setpoint, desired_lateral_jerk, CS.vEgo)

    # The extension is a passthrough here (jerk-aware + NNLC off), but call it for parity
    # with the upstream path and so the jerk/NN telemetry fields stay populated.
    pid_log, output_torque = ctl.extension.update(
      CS, VM, ctl.pid, params, ff, pid_log, setpoint, measurement, calibrated_pose, roll_compensation,
      future_desired_lateral_accel, measurement, lateral_accel_deadzone, gravity_adjusted_future_lateral_accel,
      desired_curvature, measured_curvature, steer_limited_by_safety, output_torque)
    return output_torque
