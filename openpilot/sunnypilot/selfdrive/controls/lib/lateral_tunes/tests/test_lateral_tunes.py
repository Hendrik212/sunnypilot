"""
Regression tests for the lateral tune profile layer.

Covers the two things the refactor must guarantee:
  * the upstream path selects NO profile and is untouched;
  * the StarPilot profile owns its baseline and refuses torqued's live params.
"""
import numpy as np

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
from openpilot.sunnypilot.selfdrive.controls.lib import latcontrol_ioniq6_tune as i6
from openpilot.sunnypilot.selfdrive.controls.lib.lateral_tunes import ioniq6_starpilot as i6p


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
    expected = IONIQ6_STARPILOT_TORQUE['LAT_ACCEL_FACTOR'] * i6.IONIQ_6_BASE_LAT_ACCEL_FACTOR_MULT
    assert np.isclose(ctl.torque_params.latAccelFactor, expected), ctl.torque_params.latAccelFactor
    assert np.isclose(ctl.torque_params.friction, IONIQ6_STARPILOT_TORQUE['FRICTION'])
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
    assert np.isclose(ctl.torque_params.friction, 0.09)

  def test_live_torque_params_still_apply_on_upstream(self):
    ctl, _, _ = _make_controller(HONDA.HONDA_CIVIC, starpilot=False)
    ctl.update_torque_parameters(2.75, 0.1, 0.12)
    assert np.isclose(ctl.torque_params.latAccelFactor, 2.75)
    assert np.isclose(ctl.torque_params.friction, 0.12)

  def test_lat_delay_offset_matches_starpilot(self):
    assert np.isclose(i6p.Ioniq6StarPilotProfile.lat_delay_offset, 0.1)

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
    tuned_slow = CarControllerParams(CP, 10.0, CP_SP=CP_SP)
    tuned_fast = CarControllerParams(CP, 25.0, CP_SP=CP_SP)
    assert tuned_slow.STEER_MAX == 409
    assert tuned_slow.STEER_DRIVER_ALLOWANCE == 75 and tuned_slow.STEER_THRESHOLD == 100
    assert (tuned_slow.STEER_DELTA_UP, tuned_slow.STEER_DELTA_DOWN) == (10, 8)
    assert (tuned_fast.STEER_DELTA_UP, tuned_fast.STEER_DELTA_DOWN) == (2, 3)

  def test_ioniq6_steers_at_standstill(self):
    CarInterface = interfaces[HYUNDAI.HYUNDAI_IONIQ_6]
    CP = CarInterface.get_non_essential_params(HYUNDAI.HYUNDAI_IONIQ_6)
    assert CP.steerAtStandstill
