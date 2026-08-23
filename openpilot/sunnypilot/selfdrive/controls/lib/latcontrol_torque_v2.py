import math
import numpy as np
from collections import deque

from openpilot.cereal import log
from opendbc.car.lateral import get_friction
from opendbc.sunnypilot.car.hyundai.values import IONIQ6_STARPILOT_TORQUE, is_starpilot_lat_tune
from openpilot.common.params import Params
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.common.pid import PIDController

from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_ext import LatControlTorqueExt

# At higher speeds (25+mph) we can assume:
# Lateral acceleration achieved by a specific car correlates to
# torque applied to the steering rack. It does not correlate to
# wheel slip, or to speed.

# This controller applies torque to achieve desired lateral
# accelerations. To compensate for the low speed effects the
# proportional gain is increased at low speeds by the PID controller.
# Additionally, there is friction in the steering wheel that needs
# to be overcome to move it at all, this is compensated for too.

# v2 is v0 plus a speed-scheduled low-pass on desired_curvature that strips
# the model's curvature ripple (measured 0.78 Hz on the Ioniq 6 at 123 km/h)
# before it reaches the setpoint/feedforward split. It is not car-specific:
# any vehicle that selects this tune gets the filter, which is inert below
# the blend-in speed. measurement is the loop's only feedback term, so this
# cannot move phase margin.

KP = 1.0
KI = 0.3
KD = 0.0
INTERP_SPEEDS = [1, 1.5, 2.0, 3.0, 5, 7.5, 10, 15, 30]
KP_INTERP = [250, 120, 65, 30, 11.5, 5.5, 3.5, 2.0, KP]

LP_FILTER_CUTOFF_HZ = 1.2
LAT_ACCEL_REQUEST_BUFFER_SECONDS = 1.0
FRICTION_THRESHOLD = 0.3
VERSION = 2

# Curvature-ripple prefilter (reference-side only). Cutoff is speed-scheduled
# so the filter is inert below ~72 km/h; blended in over 20-28 m/s.
CURVATURE_RIPPLE_FILTER_CUTOFF_HZ = 0.3
CURVATURE_RIPPLE_FILTER_SPEED_BP = [20.0, 28.0]  # m/s: off below, full above
CURVATURE_RIPPLE_FILTER_BLEND_V = [0.0, 1.0]

# --- StarPilot Ioniq 6 shaping shared machinery (see latcontrol_ioniq6_tune.py) ---
# These are the pieces of StarPilot's latcontrol_torque that the Ioniq 6 shaping
# hooks depend on but v2 did not have. Ported verbatim; only active when the
# LAT_TUNE_STARPILOT flag is set, so the upstream path is byte-for-byte unchanged.
MAX_LAT_JERK_UP = 2.5             # m/s^3
JERK_GAIN = 0.22
MIN_LATERAL_CONTROL_SPEED = 0.3   # m/s
STEER_RELEASE_I_DECAY = 0.8
UNWIND_D_DES_THRESHOLD = -1.0     # m/s^3
UNWIND_LAT_ACCEL_NEAR_ZERO = 0.3  # m/s^2
FF_ROLL_OFFSET_FADE_BP = [0.5, 2.5]  # m/s
FF_ROLL_OFFSET_FADE_V = [0.0, 1.0]
LOW_SPEED_X = [0, 10, 20, 30]
LOW_SPEED_Y = [12, 10.5, 8, 5]
CENTER_CHATTER_JERK_DEADZONE_SPEED_BP = [0.0, 5.0, 12.0, 25.0]  # m/s
CENTER_CHATTER_JERK_DEADZONE_SPEED_V = [0.08, 0.12, 0.18, 0.18]  # m/s^3
CENTER_CHATTER_JERK_DEADZONE_LAT_ACCEL_BP = [0.0, 0.18, 0.35]   # m/s^2
CENTER_CHATTER_JERK_DEADZONE_LAT_ACCEL_V = [1.0, 1.0, 0.0]


def get_center_chatter_friction_jerk_deadzone(v_ego, setpoint, vehicle_deadzone=0.0):
  """Return the small-signal jerk deadzone without changing turn commands."""
  speed_deadzone = np.interp(max(v_ego, 0.0), CENTER_CHATTER_JERK_DEADZONE_SPEED_BP,
                             CENTER_CHATTER_JERK_DEADZONE_SPEED_V)
  center_weight = np.interp(abs(setpoint), CENTER_CHATTER_JERK_DEADZONE_LAT_ACCEL_BP,
                            CENTER_CHATTER_JERK_DEADZONE_LAT_ACCEL_V)
  return max(float(vehicle_deadzone), float(speed_deadzone * center_weight))


class LatControlTorque(LatControl):
  def __init__(self, CP, CP_SP, CI, dt):
    super().__init__(CP, CP_SP, CI, dt)
    self.torque_params = CP.lateralTuning.torque.as_builder()
    self.torque_from_lateral_accel = CI.torque_from_lateral_accel()
    self.lateral_accel_from_torque = CI.lateral_accel_from_torque()
    self.pid = PIDController([INTERP_SPEEDS, KP_INTERP], KI, KD, rate=1/self.dt)
    self.update_limits()
    self.steering_angle_deadzone_deg = self.torque_params.steeringAngleDeadzoneDeg
    self.lat_accel_request_buffer_len = int(LAT_ACCEL_REQUEST_BUFFER_SECONDS / self.dt)
    self.lat_accel_request_buffer = deque([0.] * self.lat_accel_request_buffer_len , maxlen=self.lat_accel_request_buffer_len)
    self.previous_measurement = 0.0
    self.measurement_rate_filter = FirstOrderFilter(0.0, 1 / (2 * np.pi * LP_FILTER_CUTOFF_HZ), self.dt)
    self.curvature_ripple_filter = FirstOrderFilter(
      0.0, 1 / (2 * np.pi * CURVATURE_RIPPLE_FILTER_CUTOFF_HZ), self.dt)

    self.extension = LatControlTorqueExt(self, CP, CP_SP, CI)

    # StarPilot Ioniq 6 shaping. Only the 2023 firmware path is ported (see
    # latcontrol_ioniq6_tune.py). LateralJerkTorqueController must be off when
    # this is active: that extension replaces pid_log/output_torque with a
    # torque-space recompute using a hard-coded FRICTION_THRESHOLD and no
    # friction_scale, so StarPilot's friction shaping has no home there. The
    # passthrough-when-off behaviour of extension.update() means leaving the
    # extension call in place is correct -- the shaping flows through untouched.
    # Derived from TorqueControlTune, the single source of truth (see
    # is_starpilot_lat_tune). NOT read from a stored CP_SP flag: controlsd rebuilds this
    # controller on a live tune change, and the CarParams blob it holds was loaded at
    # process start, so a stored flag could be stale.
    _params = Params()
    self.is_ioniq6_starpilot = is_starpilot_lat_tune(CP, _params.get("TorqueControlTune"),
                                                     _params.get_bool("EnforceTorqueControl"))
    if self.is_ioniq6_starpilot:
      from openpilot.sunnypilot.selfdrive.controls.lib import latcontrol_ioniq6_tune as i6
      self._i6 = i6
      # The controller owns the StarPilot baseline rather than inheriting it from CP.
      # CP is written once at fingerprint time, so on a live switch from upstream it
      # still holds the UPSTREAM factor -- multiplying that by 1.22 would give a gain
      # that is neither tune. Setting the baseline here makes the switch stateless.
      #
      # The 1.22 multiplier is the dominant gain term and easy to miss: StarPilot's
      # effective latAccelFactor is 3.0 x 1.22 = 3.66, giving 409/3.66 = 112 CAN per
      # m/s^2 vs a measured plant-neutral ~105. Without it the tune lands 22% hot.
      self.torque_params.latAccelFactor = IONIQ6_STARPILOT_TORQUE['LAT_ACCEL_FACTOR'] * i6.IONIQ_6_BASE_LAT_ACCEL_FACTOR_MULT
      self.torque_params.friction = IONIQ6_STARPILOT_TORQUE['FRICTION']
      self.update_limits()
      # StarPilot stores curvature in the request buffer (scales by v^2 on read);
      # v2 stored lateral accel directly. Storing lateral accel makes the delayed
      # request lag the measurement whenever speed is changing, which at creep-speed
      # gains reads as a phantom unwind error during pull-away.
      self.curvature_request_buffer = deque([0.] * self.lat_accel_request_buffer_len, maxlen=self.lat_accel_request_buffer_len)
      self.jerk_filter = FirstOrderFilter(0.0, 1 / (2 * np.pi * LP_FILTER_CUTOFF_HZ), self.dt)
      self.ioniq_6_directional_taper_filter = FirstOrderFilter(1.0, i6.IONIQ_6_DIRECTIONAL_TAPER_FILTER_RC, self.dt)
      self.measurement_rate_filter = FirstOrderFilter(0.0, 1 / (2 * np.pi * (MAX_LAT_JERK_UP - 0.5)), self.dt)
      self.low_speed_reset_threshold = max(CP.minSteerSpeed, MIN_LATERAL_CONTROL_SPEED)
      self.low_speed_reset_threshold = min(self.low_speed_reset_threshold, i6.IONIQ_6_LOW_SPEED_PID_RESET_SPEED)
      self.steer_release_i_decay = STEER_RELEASE_I_DECAY
      self.prev_steering_pressed = False
      self.prev_desired_lateral_accel = 0.0
    else:
      self._i6 = None

  def update_torque_parameters(self, latAccelFactor, latAccelOffset, friction):
    # torqued fits measured lateral accel against actuatorsOutput.torque, so what it learns
    # is a property of the PLANT, not of the tune -- the same car measures the same way on
    # either tune. It is therefore used directly, exactly as on the upstream path.
    #
    # StarPilot applies its 1.22 here too, so live-learned params stay on the same gain
    # scale as the offline baseline: torqued learns the *unmultiplied* factor and the
    # controller multiplies on use, keeping the sanity window and the feedforward on one
    # consistent normalization.
    #
    # NOTE: torqued's sanity window and restore key come from CP (torqued.py), which this
    # design deliberately no longer mutates -- so both are on the upstream override.toml
    # baseline regardless of tune. The window [1.75, 3.25] comfortably contains the plant's
    # true value, so this costs nothing in practice; see the StarPilot friction floor
    # handled by MIN_FRICTION_CEILING.
    if self.is_ioniq6_starpilot:
      latAccelFactor *= self._i6.IONIQ_6_BASE_LAT_ACCEL_FACTOR_MULT
    self.torque_params.latAccelFactor = latAccelFactor
    self.torque_params.latAccelOffset = latAccelOffset
    self.torque_params.friction = friction
    self.update_limits()

  def update_limits(self):
    self.pid.set_limits(self.lateral_accel_from_torque(self.steer_max, self.torque_params),
                        self.lateral_accel_from_torque(-self.steer_max, self.torque_params))

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, calibrated_pose, curvature_limited, lat_delay):
    # Override torque params from extension
    if self.extension.update_override_torque_params(self.torque_params):
      self.update_limits()

    pid_log = log.ControlsState.LateralTorqueState.new_message()
    pid_log.version = VERSION
    # Strip the model's curvature ripple before it becomes torque. Applied here,
    # ahead of the setpoint/feedforward split, so both act on the same command.
    # measurement is the loop's only feedback term, so this cannot move phase
    # margin. Always stepped so the state stays primed across the blend-in.
    ripple_filter_input = desired_curvature
    ripple_filtered = self.curvature_ripple_filter.update(ripple_filter_input)
    ripple_blend = np.interp(CS.vEgo, CURVATURE_RIPPLE_FILTER_SPEED_BP,
                             CURVATURE_RIPPLE_FILTER_BLEND_V)
    desired_curvature = ripple_filter_input + ripple_blend * (ripple_filtered - ripple_filter_input)
    if not active:
      output_torque = 0.0
      pid_log.active = False
      # Prime with the unfiltered command (tracks measured curvature while
      # inactive) so re-engaging does not start a time-constant behind.
      self.curvature_ripple_filter.x = ripple_filter_input
      if self.is_ioniq6_starpilot:
        # Keep the request buffer and directional taper primed with the live
        # command so re-engaging with a wound wheel does not start a time-
        # constant behind (same reasoning as the ripple filter above).
        self.curvature_request_buffer.append(desired_curvature)
        self.previous_measurement = 0.0
        self.measurement_rate_filter.x = 0.0
        self.jerk_filter.x = 0.0
        self.prev_desired_lateral_accel = desired_curvature * CS.vEgo ** 2
        self.ioniq_6_directional_taper_filter.x = 1.0
    else:
      measured_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
      measurement = measured_curvature * CS.vEgo ** 2

      if self.is_ioniq6_starpilot:
        output_torque = self._update_ioniq6(active, CS, VM, params, steer_limited_by_safety,
                                            desired_curvature, measured_curvature, measurement,
                                            pid_log, lat_delay)
      else:
        output_torque = self._update_upstream(active, CS, VM, params, steer_limited_by_safety,
                                              desired_curvature, measured_curvature, measurement,
                                              pid_log, lat_delay, curvature_limited)

      pid_log.active = True
      pid_log.p = float(self.pid.p)
      pid_log.i = float(self.pid.i)
      pid_log.d = float(self.pid.d)
      pid_log.f = float(self.pid.f)
      pid_log.output = float(-output_torque) # TODO: log lat accel?
      pid_log.saturated = bool(self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited_by_safety, curvature_limited))

    # TODO left is positive in this convention
    return -output_torque, 0.0, pid_log

  def _update_upstream(self, active, CS, VM, params, steer_limited_by_safety,
                       desired_curvature, measured_curvature, measurement,
                       pid_log, lat_delay, curvature_limited):
    """The original v2 path, byte-for-byte. Only the pid_log fields that the upstream
    controller set are filled here; the caller writes p/i/d/f/output/saturated."""
    roll_compensation = params.roll * ACCELERATION_DUE_TO_GRAVITY
    curvature_deadzone = abs(VM.calc_curvature(math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0))
    lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2

    delay_frames = int(np.clip(lat_delay / self.dt, 1, self.lat_accel_request_buffer_len))
    expected_lateral_accel = self.lat_accel_request_buffer[-delay_frames]
    future_desired_lateral_accel = desired_curvature * CS.vEgo ** 2
    self.lat_accel_request_buffer.append(future_desired_lateral_accel)
    gravity_adjusted_future_lateral_accel = future_desired_lateral_accel - roll_compensation
    desired_lateral_jerk = (future_desired_lateral_accel - expected_lateral_accel) / max(lat_delay, self.dt)

    measurement_rate = self.measurement_rate_filter.update((measurement - self.previous_measurement) / self.dt)
    self.previous_measurement = measurement

    setpoint = lat_delay * desired_lateral_jerk + expected_lateral_accel
    error = setpoint - measurement

    pid_log.error = float(error)
    pid_log.actualLateralAccel = float(measurement)
    pid_log.desiredLateralAccel = float(setpoint)
    pid_log.desiredLateralJerk = float(desired_lateral_jerk)
    ff = gravity_adjusted_future_lateral_accel
    ff -= self.torque_params.latAccelOffset
    ff += get_friction(error, lateral_accel_deadzone, FRICTION_THRESHOLD, self.torque_params)

    freeze_integrator = steer_limited_by_safety or CS.steeringPressed or CS.vEgo < 5
    output_lataccel = self.pid.update(pid_log.error,
                                      -measurement_rate,
                                       feedforward=ff,
                                       speed=CS.vEgo,
                                       freeze_integrator=freeze_integrator)
    output_torque = self.torque_from_lateral_accel(output_lataccel, self.torque_params)

    # Lateral acceleration torque controller extension updates
    # Overrides pid_log.error and output_torque (passthrough when jerk-aware + NNLC off)
    pid_log, output_torque = self.extension.update(CS, VM, self.pid, params, ff, pid_log, setpoint, measurement, calibrated_pose, roll_compensation,
                                                   future_desired_lateral_accel, measurement, lateral_accel_deadzone, gravity_adjusted_future_lateral_accel,
                                                   desired_curvature, measured_curvature, steer_limited_by_safety, output_torque)
    return output_torque

  def _update_ioniq6(self, active, CS, VM, params, steer_limited_by_safety,
                     desired_curvature, measured_curvature, measurement,
                     pid_log, lat_delay):
    """StarPilot Ioniq 6 shaping, ported from StarPilot's latcontrol_torque.py (2023 path).
    Runs in lateral-acceleration space, same as upstream v2, so the multiplicative ff
    and output_torque scales convert cleanly to torque at the end. The extension is a
    passthrough (LateralJerkTorqueController is gated off for this tune), so the shaping
    done here is what reaches the actuator."""
    i6 = self._i6

    # steering-release I decay: bleed integrator when the driver lets go, so a resumed
    # hold does not start from a wound-up state.
    if self.prev_steering_pressed and not CS.steeringPressed:
      self.pid.i *= self.steer_release_i_decay
    self.prev_steering_pressed = CS.steeringPressed

    # Roll compensation fades in below walking pace; an unfaded road-crown term
    # dominates the whole feedforward at pull-away and actively unwinds a held wheel.
    roll_offset_fade = np.interp(CS.vEgo, FF_ROLL_OFFSET_FADE_BP, FF_ROLL_OFFSET_FADE_V)
    roll_compensation = params.roll * ACCELERATION_DUE_TO_GRAVITY * roll_offset_fade
    curvature_deadzone = abs(VM.calc_curvature(math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0))
    lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2

    # Curvature-space request buffer (StarPilot): scales by v^2 on read so the delayed
    # request does not lag the measurement when speed is changing.
    delay_frames = int(np.clip(lat_delay / self.dt, 1, self.lat_accel_request_buffer_len))
    expected_lateral_accel = self.curvature_request_buffer[-delay_frames] * CS.vEgo ** 2
    self.curvature_request_buffer.append(desired_curvature)
    future_desired_lateral_accel = desired_curvature * CS.vEgo ** 2

    raw_lateral_jerk = (future_desired_lateral_accel - expected_lateral_accel) / max(lat_delay, self.dt)
    raw_lateral_jerk = np.clip(raw_lateral_jerk, -MAX_LAT_JERK_UP, MAX_LAT_JERK_UP)
    desired_lateral_jerk = np.clip(self.jerk_filter.update(raw_lateral_jerk), -MAX_LAT_JERK_UP, MAX_LAT_JERK_UP)

    gravity_adjusted_future_lateral_accel = future_desired_lateral_accel - roll_compensation
    setpoint = expected_lateral_accel + desired_lateral_jerk * lat_delay
    desired_lateral_accel_rate = (setpoint - self.prev_desired_lateral_accel) / self.dt
    # Unwind detection: when the desired accel is near zero and decreasing, the car is
    # coming out of a curve. Freezing the integrator here prevents windup against the
    # soon-to-be-reduced command -- the core of the "slow curve unwind" complaint.
    unwind_detected = (desired_lateral_accel_rate < UNWIND_D_DES_THRESHOLD and
                       abs(setpoint) < UNWIND_LAT_ACCEL_NEAR_ZERO)
    self.prev_desired_lateral_accel = setpoint

    measurement_rate = self.measurement_rate_filter.update((measurement - self.previous_measurement) / self.dt)
    measurement_rate = np.clip(measurement_rate, -MAX_LAT_JERK_UP, MAX_LAT_JERK_UP)
    self.previous_measurement = measurement

    # Low-speed factor boosts the proportional gain at creep; applied to the error so
    # the friction term (which feeds on error) sees it too.
    low_speed_factor = (np.interp(CS.vEgo, LOW_SPEED_X, LOW_SPEED_Y) / max(CS.vEgo, MIN_LATERAL_CONTROL_SPEED)) ** 2
    current_kp = np.interp(CS.vEgo, self.pid._k_p[0], self.pid._k_p[1])
    error = setpoint - measurement
    error_with_lsf = error * (1 + low_speed_factor / max(current_kp, 1e-3))

    pid_log.error = float(error_with_lsf)
    pid_log.actualLateralAccel = float(measurement)
    pid_log.desiredLateralAccel = float(setpoint)
    pid_log.desiredLateralJerk = float(desired_lateral_jerk)

    # --- feedforward ---
    ff = gravity_adjusted_future_lateral_accel
    ff -= self.torque_params.latAccelOffset * roll_offset_fade

    ioniq_6_center_taper = i6.get_ioniq_6_center_taper_scale(setpoint, CS.vEgo)
    # Smooth the directional taper so jerk-gated unwind cuts can't step the FF in one frame.
    ioniq_6_directional_taper = self.ioniq_6_directional_taper_filter.update(
      i6.get_ioniq_6_directional_taper_scale(setpoint, desired_lateral_jerk, CS.vEgo))
    ff *= i6.get_ioniq_6_ff_scale(setpoint, desired_lateral_jerk, CS.vEgo,
                                  directional_taper_scale=ioniq_6_directional_taper) * ioniq_6_center_taper
    # 2023 path only: the 2025 unwind scale is gated out (see is_ioniq_6_2025_model).
    ff *= i6.get_ioniq_6_2023_unwind_ff_scale(setpoint, measurement, desired_lateral_jerk, CS.vEgo)

    # --- friction ---
    friction_threshold = i6.get_ioniq_6_friction_threshold(CS.vEgo, setpoint, desired_lateral_jerk) / max(ioniq_6_center_taper, 1e-3)
    friction_scale = i6.get_ioniq_6_friction_scale(CS.vEgo, setpoint, desired_lateral_jerk)
    friction_scale = 1.0 + ((friction_scale - 1.0) * ioniq_6_center_taper)
    friction_scale *= i6.get_ioniq_6_friction_center_fade_scale(setpoint, CS.vEgo)
    friction_jerk_deadzone = get_center_chatter_friction_jerk_deadzone(
      CS.vEgo, setpoint, i6.IONIQ_6_FRICTION_JERK_DEADZONE)
    friction_jerk = math.copysign(max(abs(desired_lateral_jerk) - friction_jerk_deadzone, 0.0),
                                  desired_lateral_jerk)
    ff += friction_scale * get_friction(error_with_lsf + JERK_GAIN * friction_jerk,
                                        lateral_accel_deadzone, friction_threshold, self.torque_params)

    # --- PID + output ---
    if CS.vEgo < self.low_speed_reset_threshold:
      self.pid.reset()
    freeze_integrator = (steer_limited_by_safety or CS.steeringPressed or
                         CS.vEgo < self.low_speed_reset_threshold or unwind_detected)
    output_lataccel = self.pid.update(pid_log.error, error_rate=-measurement_rate,
                                      feedforward=ff, speed=CS.vEgo,
                                      freeze_integrator=freeze_integrator)
    output_torque = self.torque_from_lateral_accel(output_lataccel, self.torque_params)

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

    # The extension is a passthrough here (jerk-aware + NNLC off), but call it for
    # parity with the upstream path and so the jerk/NN telemetry fields stay populated.
    pid_log, output_torque = self.extension.update(CS, VM, self.pid, params, ff, pid_log, setpoint, measurement, calibrated_pose, roll_compensation,
                                                   future_desired_lateral_accel, measurement, lateral_accel_deadzone, gravity_adjusted_future_lateral_accel,
                                                   desired_curvature, measured_curvature, steer_limited_by_safety, output_torque)
    return output_torque
