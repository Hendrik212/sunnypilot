import numpy as np
import pytest

import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom
from openpilot.sunnypilot.selfdrive.selfdrived.events import EVENTS_SP, EventsSP, corner_speed_alert
from openpilot.sunnypilot.selfdrive.selfdrived.events_base import ET

from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib import corner_speed_warning as csw
from openpilot.sunnypilot.selfdrive.controls.lib.corner_speed_warning import CornerSpeedWarning, preview_curvature

T = np.array([0.0, 0.01, 0.04, 0.09, 0.16, 0.25, 0.35, 0.48, 0.63, 0.79, 0.98, 1.18, 1.41, 1.65, 1.91, 2.2, 2.5, 2.83, 3.16, 3.5, 3.9,
              4.3, 4.7, 5.2, 5.6, 6.1, 6.6, 7.2, 7.7, 8.3, 8.9, 9.4, 10.0])
LIMIT = 2.8


def plan(v: float, radius: float, t_start: float = 0.0, t_end: float = 99.0):
  yaw_rate = np.where((T >= t_start) & (T <= t_end), v / radius, 0.0)
  return T, yaw_rate, np.full_like(T, v)


class FakeParams:
  def __init__(self, value):
    self.value = value

  def get(self, key, return_default=False):
    assert key == "LatCornerSpeedWarnAccel"
    return self.value


@pytest.fixture
def warning(mocker):
  mocker.patch.object(csw, "Params", return_value=FakeParams(LIMIT))
  return CornerSpeedWarning()


def run(w: CornerSpeedWarning, curvature: float, v_ego: float, frames: int, allowed: bool = True, limit: float = LIMIT) -> list[bool]:
  return [w.step(curvature, v_ego, allowed, limit) for _ in range(frames)]


class TestPreviewCurvature:
  def test_constant_radius(self):
    assert preview_curvature(*plan(12.0, 30.0)) == pytest.approx(1 / 30.0)

  def test_left_and_right_alike(self):
    t, yaw_rate, v = plan(12.0, 30.0)
    assert preview_curvature(t, -yaw_rate, v) == pytest.approx(1 / 30.0)

  def test_corner_beyond_the_window_is_ignored(self):
    assert preview_curvature(*plan(12.0, 30.0, t_start=csw.T_MAX + 0.3)) == 0.0

  def test_corner_only_under_the_car_is_ignored(self):
    assert preview_curvature(*plan(12.0, 30.0, t_end=csw.T_MIN - 0.06)) == 0.0

  def test_slow_plan_does_not_blow_up(self):
    t, yaw_rate, _ = plan(12.0, 30.0)
    assert preview_curvature(t, yaw_rate, np.zeros_like(t)) == pytest.approx(12.0 / 30.0 / csw.MIN_PLAN_SPEED)

  @pytest.mark.parametrize("t, yaw_rate, v", [([], [], []), ([0.5], [0.1], [10.0]), (T, T[:1], T)])
  def test_short_or_empty_plan(self, t, yaw_rate, v):
    assert preview_curvature(np.array(t), np.array(yaw_rate), np.array(v)) == 0.0

  def test_nan_plan(self):
    t, yaw_rate, v = plan(12.0, 30.0)
    assert preview_curvature(t, yaw_rate * np.nan, v) == 0.0


class TestStep:
  def test_off_by_default_limit(self, warning):
    assert not any(run(warning, 1 / 20.0, 12.0, 40, limit=0.0))

  def test_fires_after_on_frames(self, warning):
    out = run(warning, 1 / 30.0, 12.0, 10)  # 4.8 m/s^2
    assert out[:csw.ON_FRAMES - 1] == [False] * (csw.ON_FRAMES - 1)
    assert all(out[csw.ON_FRAMES - 1:])

  def test_safe_speed_is_what_the_steering_holds(self, warning):
    run(warning, 1 / 30.0, 12.0, 10)
    assert warning.safe_speed == pytest.approx(np.sqrt(LIMIT * 30.0))
    assert warning.preview_lat_accel == pytest.approx(144.0 / 30.0)

  def test_silent_below_the_limit(self, warning):
    assert not any(run(warning, 1 / 60.0, 12.0, 60))  # 2.4 m/s^2

  def test_single_frame_spike_is_ignored(self, warning):
    out = []
    for i in range(40):
      out.append(warning.step(1 / 30.0 if i % 3 == 0 else 1 / 100.0, 12.0, True, LIMIT))
    assert not any(out)

  def test_holds_through_a_short_dip_and_ends_after_off_time(self, warning):
    run(warning, 1 / 30.0, 12.0, 10)
    off_frames = int(round(csw.OFF_S / DT_MDL))
    out = run(warning, 1 / 100.0, 12.0, off_frames + 2)
    assert all(out[:off_frames - 1])
    assert not out[-1]
    assert warning.safe_speed == 0.0

  def test_stays_on_between_limit_and_release(self, warning):
    run(warning, 1 / 30.0, 12.0, 10)
    v = np.sqrt(0.95 * LIMIT * 30.0)  # below the limit, above OFF_RATIO
    assert all(run(warning, 1 / 30.0, float(v), 60))

  def test_ends_when_the_driver_slows_down(self, warning):
    run(warning, 1 / 30.0, 12.0, 10)
    out = run(warning, 1 / 30.0, 8.0, 40)  # 2.1 m/s^2
    assert not out[-1]

  def test_safe_speed_only_goes_down_during_an_alert(self, warning):
    run(warning, 1 / 40.0, 12.0, 10)
    first = warning.safe_speed
    run(warning, 1 / 25.0, 12.0, 5)
    tighter = warning.safe_speed
    run(warning, 1 / 40.0, 12.0, 5)
    assert tighter < first
    assert warning.safe_speed == tighter

  @pytest.mark.parametrize("allowed, v_ego", [(False, 12.0), (True, csw.MIN_SPEED - 0.5)])
  def test_not_allowed_or_slow(self, warning, allowed, v_ego):
    assert not any(run(warning, 1 / 5.0, v_ego, 40, allowed=allowed))

  def test_ends_at_once_when_lateral_goes_off(self, warning):
    run(warning, 1 / 30.0, 12.0, 10)
    assert not warning.step(1 / 30.0, 12.0, False, LIMIT)
    assert not warning.active
    # and needs the full ON_FRAMES again
    assert not warning.step(1 / 30.0, 12.0, True, LIMIT)


class TestParam:
  @pytest.mark.parametrize("value, expected", [(0.0, 0.0), (-1.0, 0.0), (None, 0.0), ("x", 0.0), (float("nan"), 0.0),
                                               (2.8, 2.8), (1.0, csw.LIMIT_MIN), (9.0, csw.LIMIT_MAX)])
  def test_limit_from_param(self, mocker, value, expected):
    mocker.patch.object(csw, "Params", return_value=FakeParams(value))
    assert CornerSpeedWarning().limit == expected


def messages(v: float, radius: float, lat_active: bool = True, blinker: bool = False) -> dict:
  model = messaging.new_message('modelV2').modelV2
  model.orientationRate.t = T.tolist()
  model.orientationRate.z = [v / radius] * len(T)
  # a quarter circle and on: the path leaves the x axis
  heading = np.minimum(T * v / radius, np.pi / 2)
  model.velocity.t = T.tolist()
  model.velocity.x = (v * np.cos(heading)).tolist()
  model.velocity.y = (v * np.sin(heading)).tolist()
  cs = messaging.new_message('carState').carState
  cs.vEgo = v
  cs.leftBlinker = blinker
  cc = messaging.new_message('carControl').carControl
  cc.latActive = lat_active
  return {'modelV2': model, 'carState': cs, 'carControl': cc}


class TestUpdate:
  EVENT = custom.OnroadEventSP.EventName.cornerSpeed

  def test_event_from_messages(self, warning):
    events = EventsSP()
    sm = messages(12.0, 30.0)
    for _ in range(csw.ON_FRAMES):
      events.clear()
      warning.update(sm, events)
    assert events.has(self.EVENT)
    assert warning.safe_speed == pytest.approx(np.sqrt(LIMIT * 30.0), rel=1e-3)

  @pytest.mark.parametrize("kwargs", [{"lat_active": False}, {"blinker": True}])
  def test_no_event_when_not_allowed(self, warning, kwargs):
    events = EventsSP()
    for _ in range(10):
      warning.update(messages(12.0, 30.0, **kwargs), events)
    assert not events.has(self.EVENT)

  def test_empty_model_message(self, warning):
    events = EventsSP()
    sm = messages(12.0, 30.0)
    sm['modelV2'] = messaging.new_message('modelV2').modelV2
    for _ in range(10):
      warning.update(sm, events)
    assert not events.has(self.EVENT)

  @pytest.mark.parametrize("metric, text", [(True, "Slow to 32 km/h"), (False, "Slow to 20 mph")])
  def test_alert_text(self, metric, text):
    plan = messaging.new_message('longitudinalPlanSP').longitudinalPlanSP
    plan.cornerSpeedWarning.safeSpeed = 9.1  # 32.76 km/h, 20.36 mph
    alert = corner_speed_alert(None, None, {'longitudinalPlanSP': plan}, metric, 0, None)
    assert alert.alert_text_1 == "Corner Too Fast for Steering"
    assert alert.alert_text_2 == text

  def test_event_is_registered_as_permanent(self):
    assert EVENTS_SP[self.EVENT][ET.PERMANENT] is corner_speed_alert
