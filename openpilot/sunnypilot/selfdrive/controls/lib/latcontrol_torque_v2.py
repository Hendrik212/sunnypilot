import math
import numpy as np
from collections import deque

from openpilot.cereal import log
from opendbc.car.lateral import get_friction
from openpilot.common.params import Params
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.common.pid import PIDController

from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_ext import LatControlTorqueExt
from openpilot.sunnypilot.selfdrive.controls.lib.lateral_tunes import get_lateral_tune_profile

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
#
# A vehicle-specific tune may additionally select a lateral tune PROFILE (see
# lateral_tunes/). A profile replaces the inner loop below `_update_upstream` and owns all
# of its own constants and state. When no profile is selected (`self.profile is None`) the
# upstream path runs with no profile indirection at all.

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
    self.low_speed_reset_threshold = 0.0

    self.extension = LatControlTorqueExt(self, CP, CP_SP, CI)

    # Lateral tune profile, DERIVED from params (never a stored CP_SP flag): controlsd
    # rebuilds this controller on a live tune change, and the CarParams blob it holds was
    # loaded at process start, so a stored flag could be stale.
    self.profile = get_lateral_tune_profile(CP, CP_SP, Params())
    self.profile_id = self.profile.profile_id if self.profile is not None else "upstream"
    if self.profile is not None:
      self.profile.init_controller(self, CP, CP_SP, CI)

  def update_torque_parameters(self, latAccelFactor, latAccelOffset, friction):
    # torqued fits measured lateral accel against actuatorsOutput.torque, so what it learns
    # is a property of the PLANT, not of the tune -- the same car measures the same way on
    # either tune. It is therefore used directly on the upstream path.
    #
    # A profile may decline live params entirely (use_live_torque_params). torqued seeds its
    # filter from CP.lateralTuning.torque, i.e. from the car's override.toml entry, and
    # controlsd calls this on every frame that useParams is set -- so for a profile that
    # ships its own baseline, accepting live params silently replaces the tune with the
    # seed. See lateral_tunes/ioniq6_starpilot.py for the on-device measurement.
    if self.profile is not None:
      if not self.profile.use_live_torque_params:
        return
      self.profile.apply_live_torque_params(self, latAccelFactor, latAccelOffset, friction)
    else:
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
      if self.profile is not None:
        measured_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
        self.profile.prime_inactive(self, CS, desired_curvature, measured_curvature * CS.vEgo ** 2)
    else:
      measured_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
      measurement = measured_curvature * CS.vEgo ** 2

      if self.profile is not None:
        output_torque = self.profile.update(self, active, CS, VM, params, steer_limited_by_safety,
                                            desired_curvature, measured_curvature, measurement, calibrated_pose,
                                            pid_log, lat_delay)
      else:
        output_torque = self._update_upstream(active, CS, VM, params, steer_limited_by_safety,
                                              desired_curvature, measured_curvature, measurement, calibrated_pose,
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
                       desired_curvature, measured_curvature, measurement, calibrated_pose,
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
