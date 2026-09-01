"""
Regression tests for the lateral tune profile layer.

Covers:
  * the upstream path selects NO profile and is untouched;
  * the StarPilot profile owns its baseline, PID (KP=0.6/KI=0.35), and refuses torqued;
  * StarPilot CAN envelope is speed-scheduled (409 creep / 650 mid / 409 highway)
    and the peak matches panda max_torque; unsaturated CAN/m/s^2 stays 409/3.66.
"""
from pathlib import Path

import numpy as np
import pytest

from opendbc.car.car_helpers import interfaces
from opendbc.car.honda.values import CAR as HONDA
from opendbc.car.hyundai.values import CAR as HYUNDAI, CarControllerParams
from opendbc.car.structs import car
from opendbc.car.vehicle_model import VehicleModel
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP, IONIQ6_STARPILOT_TORQUE
from openpilot.cereal import log
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.car.helpers import convert_to_capnp
from openpilot.selfdrive.locationd.helpers import Pose
from openpilot.common.mock.generators import generate_deviceMotion
from openpilot.sunnypilot.selfdrive.car import interfaces as sunnypilot_interfaces
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v2 import LatControlTorque
from openpilot.sunnypilot.selfdrive.controls.lib.lateral_tunes import ioniq6_shaping as i6
from openpilot.sunnypilot.selfdrive.controls.lib.lateral_tunes import ioniq6_starpilot as i6p
from openpilot.sunnypilot.selfdrive.controls.lib.lateral_tunes import ripple_notch as rn


def _make_controller(car_name, starpilot: bool):
  params = Params()
  params.put_bool("EnforceTorqueControl", True, block=True)
  params.put_bool("LateralJerkTorqueController", False, block=True)
  params.put_bool("NeuralNetworkLateralControl", False, block=True)
  params.put("TorqueControlTune", 2.0 if starpilot else 0.0, block=True)

  CarInterface = interfaces[car_name]
  CP = CarInterface.get_non_essential_params(car_name)
  CP_SP = CarInterface.get_non_essential_params_sp(CP, car_name)
  CI = CarInterface(CP, CP_SP)
  sunnypilot_interfaces.setup_interfaces(CI, params)
  CP_SP = convert_to_capnp(CP_SP)
  VM = VehicleModel(CP)
  return LatControlTorque(CP.as_reader(), CP_SP.as_reader(), CI, DT_CTRL), VM, CP


def _run(controller, VM, v_ego=12.0, active=True, curvature=0.01, lat_delay=0.4585):
  CS = car.CarState.new_message()
  CS.vEgo = v_ego
  CS.steeringAngleDeg = 5.0
  CS.steeringPressed = False
  pose = Pose.from_device_motion(generate_deviceMotion().deviceMotion)
  lp = log.VehicleParameters.new_message()
  controller.extension.update_lateral_lag(lat_delay)
  return controller.update(active, CS, VM, lp, False, curvature, pose, False, lat_delay)


class TestLateralTuneProfiles(OpenpilotTestCase):
  def test_upstream_selects_no_profile(self):
    ctl, _, _ = _make_controller(HONDA.HONDA_CIVIC, starpilot=False)
    assert ctl.profile is None
    assert ctl.profile_id == "upstream"

  def test_ioniq6_selects_starpilot_profile(self):
    ctl, _, _ = _make_controller(HYUNDAI.HYUNDAI_IONIQ_6, starpilot=True)
    assert ctl.profile is not None
    assert ctl.profile_id == "ioniq6_starpilot"

  def test_ioniq6_upstream_tune_selects_no_profile(self):
    # Same car, tune switched off -> upstream path, no profile indirection.
    ctl, _, _ = _make_controller(HYUNDAI.HYUNDAI_IONIQ_6, starpilot=False)
    assert ctl.profile is None

  def test_starpilot_baseline_is_owned_by_the_profile(self):
    ctl, _, CP = _make_controller(HYUNDAI.HYUNDAI_IONIQ_6, starpilot=True)
    # The constructor applies the speed schedule at v=0, so compare against the scheduled
    # value rather than the raw base -- the schedule is the source of truth for both.
    from opendbc.sunnypilot.car.hyundai.lateral_limits import lat_accel_factor_for_speed, friction_for_speed
    base = IONIQ6_STARPILOT_TORQUE['LAT_ACCEL_FACTOR'] * i6.IONIQ_6_BASE_LAT_ACCEL_FACTOR_MULT
    expected_laf = lat_accel_factor_for_speed(0.0, base)
    expected_fr = friction_for_speed(0.0, IONIQ6_STARPILOT_TORQUE['FRICTION'])
    assert np.isclose(ctl.torque_params.latAccelFactor, expected_laf), ctl.torque_params.latAccelFactor
    assert np.isclose(ctl.torque_params.friction, expected_fr), ctl.torque_params.friction
    # and it is NOT the car's override.toml seed
    assert not np.isclose(ctl.torque_params.latAccelFactor, CP.lateralTuning.torque.latAccelFactor)

  def test_live_torque_params_cannot_overwrite_the_starpilot_tune(self):
    """torqued seeds from override.toml ([2.5, 2.5, 0.005]) and controlsd feeds that back
    every frame, replacing the tune with the seed."""
    ctl, _, _ = _make_controller(HYUNDAI.HYUNDAI_IONIQ_6, starpilot=True)
    before = (ctl.torque_params.latAccelFactor, ctl.torque_params.friction)
    ctl.update_torque_parameters(2.5, 0.0, 0.005)
    after = (ctl.torque_params.latAccelFactor, ctl.torque_params.friction)
    assert before == after, f"live params overwrote the tune: {before} -> {after}"
    # friction is the SCHEDULED value at v=0, not a hardcoded constant -- derive it from the
    # schedule so this test survives future ceiling changes.
    from opendbc.sunnypilot.car.hyundai.lateral_limits import friction_for_speed
    assert np.isclose(ctl.torque_params.friction, friction_for_speed(0.0, IONIQ6_STARPILOT_TORQUE['FRICTION']))

  def test_live_torque_params_still_apply_on_upstream(self):
    ctl, _, _ = _make_controller(HONDA.HONDA_CIVIC, starpilot=False)
    ctl.update_torque_parameters(2.75, 0.1, 0.12)
    assert np.isclose(ctl.torque_params.latAccelFactor, 2.75)
    assert np.isclose(ctl.torque_params.friction, 0.12)

  def test_lat_delay_offset_matches_starpilot(self):
    # modeld now sets LAT_SMOOTH_SECONDS = 0.1 (the source curvature LP), matching
    # StarPilot, so the profile no longer compensates -- offset is 0.0 to avoid
    # double-counting the 0.1 that controlsd already adds.
    assert np.isclose(i6p.Ioniq6StarPilotProfile.lat_delay_offset, 0.0)

  def test_low_speed_factor_floor_matches_starpilot(self):
    from openpilot.selfdrive.controls.lib.drive_helpers import MIN_SPEED
    assert np.isclose(i6p.Ioniq6StarPilotProfile.low_speed_factor_min_speed, MIN_SPEED)

    def lsf(v, floor):
      return (np.interp(v, i6p.LOW_SPEED_X, i6p.LOW_SPEED_Y) / max(v, floor)) ** 2

    assert np.isclose(lsf(0.3, MIN_SPEED), 142.92, atol=0.01)
    # Below 1 m/s the two floors diverge as (MIN_SPEED / max(v, 0.3)) ** 2: 11.1x at
    # 0.3 m/s and 4.0x at 0.5 m/s. Lateral is live from 0.3 m/s up (the standstill gate),
    # so that whole band is reachable in stop-and-go.
    for v, expected in ((0.3, 11.11), (0.5, 4.0), (0.8, 1.5625)):
      assert np.isclose(lsf(v, 0.3) / lsf(v, MIN_SPEED), expected, rtol=1e-3), v
    # at and above MIN_SPEED the floors are identical
    assert np.isclose(lsf(1.5, 0.3), lsf(1.5, MIN_SPEED))

  def test_both_paths_run_active_and_inactive(self):
    for car_name, sp in ((HONDA.HONDA_CIVIC, False), (HYUNDAI.HYUNDAI_IONIQ_6, True)):
      ctl, VM, _ = _make_controller(car_name, starpilot=sp)
      for v in (0.5, 5.0, 12.0, 25.0):
        for active in (False, True):
          torque, _, pid_log = _run(ctl, VM, v_ego=v, active=active)
          assert np.isfinite(torque), (car_name, v, active)
          assert abs(torque) <= 1.0 + 1e-6, (car_name, v, active, torque)
          assert pid_log.active == active

  def test_inactive_branch_resets_pid_and_tracks_steering_pressed(self):
    ctl, VM, _ = _make_controller(HYUNDAI.HYUNDAI_IONIQ_6, starpilot=True)
    ctl.pid.i = 0.5
    CS = car.CarState.new_message()
    CS.vEgo = 0.4
    CS.steeringPressed = True
    pose = Pose.from_device_motion(generate_deviceMotion().deviceMotion)
    ctl.update(False, CS, VM, log.VehicleParameters.new_message(), False, 0.0, pose, False, 0.4585)
    assert ctl.pid.i == 0.0
    assert ctl.profile.prev_steering_pressed is True

  def test_steer_limits_follow_the_tune_flag(self):
    CarInterface = interfaces[HYUNDAI.HYUNDAI_IONIQ_6]
    CP = CarInterface.get_non_essential_params(HYUNDAI.HYUNDAI_IONIQ_6)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, HYUNDAI.HYUNDAI_IONIQ_6)

    CP_SP.flags &= ~HyundaiFlagsSP.LAT_TUNE_STARPILOT.value
    stock = CarControllerParams(CP, 10.0, CP_SP=CP_SP)
    assert stock.STEER_MAX == 270 and stock.STEER_DELTA_UP == 2

    CP_SP.flags |= HyundaiFlagsSP.LAT_TUNE_STARPILOT.value
    tuned_creep = CarControllerParams(CP, 2.0, CP_SP=CP_SP)
    tuned_slow = CarControllerParams(CP, 10.0, CP_SP=CP_SP)
    tuned_fast = CarControllerParams(CP, 25.0, CP_SP=CP_SP)
    assert tuned_creep.STEER_MAX == 409  # 0-18 km/h stays at the StarPilot rail
    assert tuned_slow.STEER_MAX == 650 and tuned_fast.STEER_MAX == 409
    assert tuned_slow.STEER_DRIVER_ALLOWANCE == 75 and tuned_slow.STEER_THRESHOLD == 100
    assert (tuned_slow.STEER_DELTA_UP, tuned_slow.STEER_DELTA_DOWN) == (10, 8)
    assert (tuned_fast.STEER_DELTA_UP, tuned_fast.STEER_DELTA_DOWN) == (2, 3)

  def test_steer_max_schedule_and_panda_envelope(self):
    """The car layer schedules STEER_MAX inside the panda envelope. Panda must never block
    what the car layer can request, or commands are silently clipped (see
    UPSTREAM_MERGE_GUIDE: values.py and hyundai_canfd.h must stay in sync)."""
    import re
    from opendbc.sunnypilot.car.hyundai import lateral_limits as ll

    CarInterface = interfaces[HYUNDAI.HYUNDAI_IONIQ_6]
    CP = CarInterface.get_non_essential_params(HYUNDAI.HYUNDAI_IONIQ_6)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, HYUNDAI.HYUNDAI_IONIQ_6)
    CP_SP.flags |= HyundaiFlagsSP.LAT_TUNE_STARPILOT.value

    # panda's compiled ceiling, read from the safety source itself
    safety = (Path(__file__).resolve().parents[7] / 'opendbc_repo' / 'opendbc' / 'safety' /
              'modes' / 'hyundai_canfd.h').read_text()
    m = re.search(r'\.max_torque\s*=\s*(\d+)', safety)
    assert m, "could not read max_torque from hyundai_canfd.h"
    panda_max = int(m.group(1))

    worst = 0
    for v in np.arange(0.0, 40.0, 0.25):
      steer_max = CarControllerParams(CP, float(v), CP_SP=CP_SP).STEER_MAX
      worst = max(worst, steer_max)
      assert steer_max <= panda_max, f"car layer asks {steer_max} at {v} m/s, panda allows {panda_max}"
    assert worst == panda_max, f"panda envelope {panda_max} does not match peak request {worst}"

    assert CarControllerParams(CP, 0.0, CP_SP=CP_SP).STEER_MAX == 409
    assert CarControllerParams(CP, 5.0, CP_SP=CP_SP).STEER_MAX == 409
    assert CarControllerParams(CP, 6.5, CP_SP=CP_SP).STEER_MAX == 650
    assert CarControllerParams(CP, 10.0, CP_SP=CP_SP).STEER_MAX == 650
    assert CarControllerParams(CP, 15.0, CP_SP=CP_SP).STEER_MAX == 650
    assert CarControllerParams(CP, 17.0, CP_SP=CP_SP).STEER_MAX == 409
    assert CarControllerParams(CP, 30.0, CP_SP=CP_SP).STEER_MAX == 409
    assert ll.STARPILOT_STEER_MAX == panda_max
    assert ll.CANFD_STEER_MAX_SPEED_BP[-1] <= ll.CANFD_STEER_RATE_SPEED_BP[0]

  def test_upstream_canfd_limits_untouched_by_the_schedule(self):
    CarInterface = interfaces[HYUNDAI.HYUNDAI_IONIQ_6]
    CP = CarInterface.get_non_essential_params(HYUNDAI.HYUNDAI_IONIQ_6)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, HYUNDAI.HYUNDAI_IONIQ_6)
    CP_SP.flags &= ~HyundaiFlagsSP.LAT_TUNE_STARPILOT.value
    for v in (0.0, 10.0, 16.0, 30.0):
      p = CarControllerParams(CP, v, CP_SP=CP_SP)
      assert (p.STEER_MAX, p.STEER_DELTA_UP, p.STEER_DELTA_DOWN) == (270, 2, 3), v

  def test_starpilot_pid_gains_match_starpilot(self):
    """v2 constructs KP=1.0 / KI=0.3; the profile must overwrite those with StarPilot's
    0.6 / 0.35. Below 15 m/s the interp table is identical, so this only diverges on
    the highway knot -- which is exactly where an un-overwritten v2 PID is 67% hot."""
    ctl, _, _ = _make_controller(HYUNDAI.HYUNDAI_IONIQ_6, starpilot=True)
    assert np.isclose(np.interp(15.0, ctl.pid._k_p[0], ctl.pid._k_p[1]), 2.0)
    assert np.isclose(np.interp(30.0, ctl.pid._k_p[0], ctl.pid._k_p[1]), i6p.KP)
    assert np.isclose(ctl.pid._k_i[1][0], i6p.KI)
    assert np.isclose(i6p.KP, 0.6) and np.isclose(i6p.KI, 0.35)

    # upstream v2 path is untouched
    up, _, _ = _make_controller(HONDA.HONDA_CIVIC, starpilot=False)
    assert np.isclose(np.interp(30.0, up.pid._k_p[0], up.pid._k_p[1]), 1.0)
    assert np.isclose(up.pid._k_i[1][0], 0.3)

    ioniq_up, _, _ = _make_controller(HYUNDAI.HYUNDAI_IONIQ_6, starpilot=False)
    assert ioniq_up.profile is None
    assert np.isclose(np.interp(30.0, ioniq_up.pid._k_p[0], ioniq_up.pid._k_p[1]), 1.0)

  def test_lat_accel_factor_scales_with_steer_max(self):
    """Unsaturated CAN/m/s^2 must stay 409/3.66 as STEER_MAX changes."""
    from opendbc.sunnypilot.car.hyundai.lateral_limits import lat_accel_factor_for_speed, steer_max_for_speed
    base = IONIQ6_STARPILOT_TORQUE['LAT_ACCEL_FACTOR'] * i6.IONIQ_6_BASE_LAT_ACCEL_FACTOR_MULT
    assert np.isclose(base, 3.66)
    for v in (0.0, 5.0, 6.5, 10.0, 15.0, 17.0, 30.0):
      sm = steer_max_for_speed(v)
      laf = lat_accel_factor_for_speed(v, base)
      assert np.isclose(sm / laf, 409 / 3.66, rtol=1e-3), (v, sm, laf)

  def test_friction_can_is_invariant_to_steer_max(self):
    """latAccelFactor cancels out of the friction term (get_friction multiplies by it, the
    feedforward divide cancels it), so friction's CAN contribution is friction*STEER_MAX and
    scaling latAccelFactor alone does NOT hold it. It must stay 0.09*409 = 37 CAN."""
    from opendbc.sunnypilot.car.hyundai.lateral_limits import friction_for_speed, steer_max_for_speed
    base = IONIQ6_STARPILOT_TORQUE['FRICTION']
    for v in (0.0, 5.0, 6.5, 10.0, 15.0, 17.0, 30.0):
      can = friction_for_speed(v, base) * steer_max_for_speed(v)
      assert np.isclose(can, base * 409, rtol=1e-3), (v, can)

  def test_profile_schedules_friction_with_the_ceiling(self):
    """The invariance above is worthless unless the profile actually writes it each frame."""
    ctl, _, _ = _make_controller(HYUNDAI.HYUNDAI_IONIQ_6, starpilot=True)
    for v, steer_max in ((0.0, 409), (10.0, 650), (30.0, 409)):
      ctl.profile._apply_speed_scheduled_factor(ctl, v)
      assert np.isclose(ctl.torque_params.friction * steer_max, 0.09 * 409, rtol=1e-3), v
    # and it is idempotent -- repeated calls at one speed must not compound
    ctl.profile._apply_speed_scheduled_factor(ctl, 10.0)
    first = ctl.torque_params.friction
    for _ in range(5):
      ctl.profile._apply_speed_scheduled_factor(ctl, 10.0)
    assert ctl.torque_params.friction == first

  def test_low_speed_reset_threshold_is_not_degenerate(self):
    """Was min(max(minSteerSpeed, 0.3), 0.0447), which is the constant 0.0447 for every
    input -- both other terms dead. The reset must sit at the highest of the three."""
    ctl, _, CP = _make_controller(HYUNDAI.HYUNDAI_IONIQ_6, starpilot=True)
    assert np.isclose(ctl.low_speed_reset_threshold, i6p.MIN_LATERAL_CONTROL_SPEED)
    assert ctl.low_speed_reset_threshold >= CP.minSteerSpeed
    assert ctl.low_speed_reset_threshold > i6.IONIQ_6_LOW_SPEED_PID_RESET_SPEED

  def test_ioniq6_steers_at_standstill(self):
    CarInterface = interfaces[HYUNDAI.HYUNDAI_IONIQ_6]
    CP = CarInterface.get_non_essential_params(HYUNDAI.HYUNDAI_IONIQ_6)
    assert CP.steerAtStandstill

  def test_ripple_prefilter_is_owned_by_the_profile(self):
    """v2 applies no shaping of its own; the prefilter belongs to the Ioniq 6 profile."""
    import openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v2 as v2
    # The constants and the filter state must no longer live on the generic controller.
    assert not hasattr(v2, "CURVATURE_RIPPLE_FILTER_CUTOFF_HZ")
    assert not hasattr(v2, "RIPPLE_NOTCH_HZ")
    up, _, _ = _make_controller(HONDA.HONDA_CIVIC, starpilot=False)
    assert not hasattr(up, "curvature_ripple_filter")
    assert not hasattr(up, "curvature_ripple_notch")

  def test_notch_is_unity_at_dc_and_kills_its_centre(self):
    """A notch that does not pass DC would bias every steady-state corner."""
    dt = 0.01
    n = rn.NotchFilter(dt, rn.RIPPLE_NOTCH_HZ, rn.RIPPLE_NOTCH_Q)
    n.reset(0.02)
    for _ in range(2000):
      out = n.update(0.02)
    assert np.isclose(out, 0.02, rtol=1e-6), out

    for f, keep in ((rn.RIPPLE_NOTCH_HZ, 0.05), (0.05, 0.9), (3.0, 0.9)):
      n = rn.NotchFilter(dt, rn.RIPPLE_NOTCH_HZ, rn.RIPPLE_NOTCH_Q)
      t = np.arange(6000) * dt
      x = np.sin(2 * np.pi * f * t)
      y = np.array([n.update(v) for v in x])
      amp = np.std(y[3000:]) / np.std(x[3000:])
      if f == rn.RIPPLE_NOTCH_HZ:
        assert amp < keep, (f, amp)
      else:
        assert amp > keep, (f, amp)

  def test_creep_center_relief_fires_only_near_centre_at_creep(self):
    """The relief must cut the near-centre creep error and nothing else. Measured envelope
    on route 000001c5: 0.70 on the seg-22 ping-pong band, 0.000 on parking maneuvers and
    0.000 on normal driving."""
    f = i6.get_ioniq_6_2023_low_speed_center_error_scale
    S = i6.IONIQ_6_2023_LOW_SPEED_CENTER_ERROR_SCALE

    # the ping-pong case: creep, straight, model asking for nothing
    near_centre = f(-0.002, 0.0, 1.5, 6.5)
    assert near_centre < 0.5, near_centre

    # must NOT touch real steering: past the angle gate, off centre, or during a jerk
    assert f(-0.002, 0.0, 1.5, 460.0) == pytest.approx(1.0, abs=0.02)   # parking lock
    assert f(-0.90, 0.0, 1.5, 6.5) == pytest.approx(1.0, abs=0.02)      # real turn command
    assert f(-0.002, 2.0, 1.5, 6.5) == pytest.approx(1.0, abs=0.02)     # jerk commanded
    # and effectively nothing above creep speed (<=1.1% at 8 m/s, none by 15)
    assert f(-0.002, 0.0, 8.0, 6.5) > 0.98
    for v in (15.0, 30.0):
      assert f(-0.002, 0.0, v, 6.5) == pytest.approx(1.0, abs=1e-3), v

    # bounded by the scale, never inverts or amplifies
    for ds in (-1.0, -0.1, 0.0, 0.1, 1.0):
      for v in (0.0, 1.0, 3.0, 10.0):
        for ang in (0.0, 30.0, 200.0):
          assert S - 1e-9 <= f(ds, 0.0, v, ang) <= 1.0 + 1e-9

  def test_creep_center_relief_raises_the_rail_threshold(self):
    """Sizing check. seg 22 railed on a raw error of 0.066 m/s^2. The relief must raise the
    error needed to rail by >=2x across the creep band, and clear 0.066 from ~7 km/h up.

    It does NOT clear 0.066 below that: the effective creep gain (kp ~250 at 1 m/s) is too
    extreme for a bounded multiplier -- even S=0.03 only reaches 0.062 at 5.4 km/h. That is
    a known limit, recorded so a future drive that still weaves under ~7 km/h points at the
    gain table or the roll term rather than at this scale."""
    prof = i6p.Ioniq6StarPilotProfile

    def rail_error(v, scaled):
      kp = np.interp(v, i6p.INTERP_SPEEDS, i6p.KP_INTERP)
      lsf = (np.interp(v, i6p.LOW_SPEED_X, i6p.LOW_SPEED_Y) /
             max(v, prof.low_speed_factor_min_speed)) ** 2
      eff = kp * (1 + lsf / max(kp, 1e-3))
      s = i6.get_ioniq_6_2023_low_speed_center_error_scale(-0.002, 0.0, v, 6.5) if scaled else 1.0
      return 3.66 / (eff * s)

    for v in (1.0, 1.5, 2.0, 2.5, 3.0):
      assert rail_error(v, True) >= 2.0 * rail_error(v, False), v
    for v in (2.0, 2.5, 3.0):          # ~7 km/h and up
      assert rail_error(v, True) > 0.066, (v, rail_error(v, True))

  def test_monitor_measures_a_synthetic_ripple(self):
    """The monitor's one job. Feed a clean sinusoid at a known frequency and it must report
    it. This is the regression that would have caught the 0.45 Hz search floor: on Sep 1 the
    monitor logged measured_hz = 0.0 for a whole route while the offline estimator found a
    x12-x68 bump, because the big model's ripple sat below the search band."""
    for f_true in (0.30, 0.45, 0.69, 0.90):
      mon = rn.RippleFrequencyMonitor(DT_CTRL)
      n = int(180.0 / DT_CTRL)
      t = np.arange(n) * DT_CTRL
      # ripple + pink-ish background so the power-law fit has something to fit
      rng = np.random.default_rng(0)
      bg = np.cumsum(rng.normal(0.0, 2e-5, n))
      sig = 3e-3 * np.sin(2 * np.pi * f_true * t) + bg
      for v in sig:
        mon.update(float(v), 25.0)
      assert mon.measured_hz == pytest.approx(f_true, abs=0.08), (f_true, mon.measured_hz)
      assert mon.excess > rn.ESTIMATOR_MIN_EXCESS, (f_true, mon.excess)

  def test_monitor_publishes_a_reading_on_every_spectrum(self):
    """measured_hz/excess are telemetry and must be published whenever a spectrum exists;
    only `qualifying` is gated. A flat 0.0 on a route with a real bump was the bug (Sep 1:
    the monitor logged 0.0 for a whole route while the offline estimator found x12-x68).

    NB the excess gate does NOT separate ripple from noise -- pure power-law noise scores
    excess 5.7-9.7 here, versus a ~7.6 median on real qualifying windows (000001a4). What
    actually protects the notch is that nothing reads `f_hz`
    (see test_ripple_monitor_never_reaches_control) plus RIPPLE_CLAMP_HZ and the slow slew.
    Do not treat `qualifying` as evidence a real ripple was found."""
    mon = rn.RippleFrequencyMonitor(DT_CTRL)
    n = int(180.0 / DT_CTRL)
    rng = np.random.default_rng(1)
    for v in np.cumsum(rng.normal(0.0, 3e-5, n)):
      mon.update(float(v), 25.0)
    assert mon.measured_hz > 0.0, "a spectrum existed but nothing was logged"
    assert mon.excess > 0.0

  def test_monitor_search_band_covers_the_big_model_ripple(self):
    """BMV4 measures 0.40-0.47 Hz and an earlier chestnut checkpoint 0.27 Hz; the retune
    clamp stays narrow so a wide search cannot drag the notch somewhere unvalidated."""
    assert rn.RIPPLE_SEARCH_HZ[0] <= 0.25
    assert rn.RIPPLE_BACKGROUND_FIT_HZ[0] < rn.RIPPLE_SEARCH_HZ[0]
    assert rn.RIPPLE_CLAMP_HZ[0] >= 0.45 and rn.RIPPLE_CLAMP_HZ[1] <= 1.05

  def test_ripple_monitor_never_reaches_control(self):
    """The monitor is inert by construction: the notch frequency is the constant, and
    nothing on the profile feeds monitor output back into the filter. Validated offline:
    tracking the estimate made the filter WORSE (7.2% -> 21.6% of ripple kept)."""
    ctl, _, _ = _make_controller(HYUNDAI.HYUNDAI_IONIQ_6, starpilot=True)
    prof = ctl.profile
    assert np.isclose(prof.curvature_ripple_notch.f0, rn.RIPPLE_NOTCH_HZ)
    prof.ripple_monitor.f_hz = 0.95          # pretend it converged somewhere else
    prof.ripple_monitor.measured_hz = 0.95
    CS = car.CarState.new_message()
    CS.vEgo = 25.0
    prof.filter_desired_curvature(ctl, CS, 0.01, True)
    assert np.isclose(prof.curvature_ripple_notch.f0, rn.RIPPLE_NOTCH_HZ)

  def test_notch_centre_matches_the_measured_ripple(self):
    """The whole design rests on this number. Measured on route 000001a4 against a fitted
    power-law background: 14.8x excess at 0.69 Hz in BOTH desired curvature and steering
    angle, and the offline centre sweep is sharply peaked there (ripple energy kept:
    0.62 Hz 20.8%, 0.65 Hz 12.6%, 0.69 Hz 7.2%, 0.72 Hz 8.0%, 0.75 Hz 12.4%).

    Do not widen this window to accommodate a new value -- re-run the offline validation
    in ripple_notch.py's docstring against a route from the new model and move it.
    """
    assert 0.66 <= rn.RIPPLE_NOTCH_HZ <= 0.72, rn.RIPPLE_NOTCH_HZ
    assert 1.5 <= rn.RIPPLE_NOTCH_Q <= 2.5, rn.RIPPLE_NOTCH_Q
    # The notch must sit inside the band the monitor is allowed to report, or a genuine
    # model shift would be clamped out of the logs before anyone could see it.
    assert rn.RIPPLE_CLAMP_HZ[0] < rn.RIPPLE_NOTCH_HZ < rn.RIPPLE_CLAMP_HZ[1]

  def test_notch_speed_blend_covers_the_70kmh_weave(self):
    """The LP this replaced was still at blend 0.0 at 19.4 m/s (70 km/h) -- it never
    touched the band it was aimed at. The notch must be fully on there."""
    assert np.interp(19.4, rn.RIPPLE_NOTCH_SPEED_BP, rn.RIPPLE_NOTCH_BLEND_V) == 1.0
    assert np.interp(16.7, rn.RIPPLE_NOTCH_SPEED_BP, rn.RIPPLE_NOTCH_BLEND_V) == 1.0
    assert np.interp(10.0, rn.RIPPLE_NOTCH_SPEED_BP, rn.RIPPLE_NOTCH_BLEND_V) == 0.0

  def test_ripple_notch_is_inert_below_blend_in(self):
    """Below the blend the command must pass through byte-for-byte; above it the ripple
    band must actually be attenuated."""
    ctl, _, _ = _make_controller(HYUNDAI.HYUNDAI_IONIQ_6, starpilot=True)
    prof = ctl.profile
    CS = car.CarState.new_message()

    CS.vEgo = 10.0
    prof.curvature_ripple_notch.reset(0.0)
    assert prof.filter_desired_curvature(ctl, CS, 0.02, True) == 0.02

    # At 25 m/s, a sustained tone at the notch centre must come out attenuated while a
    # constant command passes untouched.
    CS.vEgo = 25.0
    prof.curvature_ripple_notch.reset(0.0)
    t = np.arange(4000) * DT_CTRL
    x = 0.01 * np.sin(2 * np.pi * rn.RIPPLE_NOTCH_HZ * t)
    y = np.array([prof.filter_desired_curvature(ctl, CS, float(v), True) for v in x])
    assert np.std(y[2000:]) / np.std(x[2000:]) < 0.1

    prof.curvature_ripple_notch.reset(0.02)
    for _ in range(500):
      out = prof.filter_desired_curvature(ctl, CS, 0.02, True)
    assert np.isclose(out, 0.02, rtol=1e-6), out

  def test_ripple_notch_primes_while_inactive(self):
    """Inactive must pin the notch to the live command, not run it."""
    ctl, _, _ = _make_controller(HYUNDAI.HYUNDAI_IONIQ_6, starpilot=True)
    CS = car.CarState.new_message()
    CS.vEgo = 30.0
    ctl.profile.curvature_ripple_notch.reset(0.0)
    out = ctl.profile.filter_desired_curvature(ctl, CS, 0.02, False)
    assert out == 0.02
    assert ctl.profile.curvature_ripple_notch.y1 == 0.02

  def test_turn_intent_hold_is_gated_on_the_profile(self):
    """turn_intent runs only for a profile that opts in -- not on the upstream path."""
    from openpilot.sunnypilot.selfdrive.controls.lib.lateral_tunes.base import LateralTuneProfile
    assert LateralTuneProfile().uses_turn_intent_hold is False

    up, _, _ = _make_controller(HONDA.HONDA_CIVIC, starpilot=False)
    assert getattr(up.profile, "uses_turn_intent_hold", False) is False

    ioniq_up, _, _ = _make_controller(HYUNDAI.HYUNDAI_IONIQ_6, starpilot=False)
    assert getattr(ioniq_up.profile, "uses_turn_intent_hold", False) is False

    ctl, _, _ = _make_controller(HYUNDAI.HYUNDAI_IONIQ_6, starpilot=True)
    assert ctl.profile.uses_turn_intent_hold is True
