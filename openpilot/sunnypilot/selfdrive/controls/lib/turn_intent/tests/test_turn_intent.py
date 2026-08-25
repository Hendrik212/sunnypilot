"""Tests for the low-speed turn-intent curvature hold."""
import numpy as np

from opendbc.car.structs import car
from openpilot.cereal import log, messaging
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_CTRL
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.controls.lib.turn_intent import TurnIntentHold
from openpilot.sunnypilot.selfdrive.controls.lib.turn_intent.constants import (
  CURVATURE_HOLD_PLAN_CAP,
  CURVATURE_HOLD_PLAN_SOURCE_SPEED,
  CURVATURE_HOLD_RELEASE_SPEED,
  CURVATURE_HOLD_STANDSTILL_TIMEOUT,
  TURN_LEAD_CAP,
  TURN_LEAD_MAX_SPEED,
)

# ys = 0.5 * curve * x^2 makes get_plan_spatial_curvature() read back ~= +curve, so a
# positive `curve` is a RIGHT turn, matching the module's sign convention (right = +).
def _model(curve=0.0, reach=40.0, lane_change=False):
  m = messaging.new_message('modelV2').modelV2
  n = len(ModelConstants.T_IDXS)
  xs = np.linspace(0.0, reach, n)
  ys = 0.5 * curve * xs ** 2
  pos = log.XYZTData.new_message()
  pos.x = [float(v) for v in xs]
  pos.y = [float(v) for v in ys]
  pos.z = [0.0] * n
  m.position = pos
  if lane_change:
    m.meta.laneChangeState = log.LaneChangeState.laneChangeStarting
  return m


def _cs(v=1.0, right=False, left=False, pressed=False, torque=0.0, a=0.0):
  cs = car.CarState.new_message()
  cs.vEgo, cs.aEgo = v, a
  cs.rightBlinker, cs.leftBlinker = right, left
  cs.steeringPressed, cs.steeringTorque = pressed, torque
  return cs


def _arm(h, curve=0.06, v=0.6, n=200):
  """The scenario this feature exists for: creeping toward a signalled turn with the
  model's action collapsed to ~0 while the plan's geometry still shows the corner."""
  m = _model(curve=curve)
  for _ in range(n):
    h.update(0.0, _cs(v=v, right=True), m, True, 0.0)
  return m


class TestTurnIntentHold(OpenpilotTestCase):
  def test_no_blinker_is_a_strict_no_op(self):
    """The guarantee that keeps this off ordinary lane keeping."""
    h = TurnIntentHold()
    rng = np.random.default_rng(7)
    for _ in range(3000):
      v, curv, cmd = rng.uniform(0, 30), rng.normal(0, 0.02), rng.normal(0, 0.02)
      out = h.update(float(cmd), _cs(v=float(v)), _model(curve=float(curv)), True, float(curv))
      assert out == float(cmd), (v, cmd, out)

  def test_plan_probe_arms_the_hold_while_the_action_is_blind(self):
    h = TurnIntentHold()
    _arm(h)
    assert h.turn_hold_curvature > 0.0, "plan-sourced pre-wind never armed"
    assert h.turn_hold_curvature <= CURVATURE_HOLD_PLAN_CAP

  def test_hold_floors_the_command_when_the_model_collapses(self):
    h = TurnIntentHold()
    m = _arm(h)
    held = h.turn_hold_curvature
    out = h.update(0.0, _cs(v=0.6, right=True), m, True, 0.0)
    assert np.isclose(out, held, rtol=1e-6), f"command was not floored: {out} vs hold {held}"

  def test_opposite_model_command_releases_immediately(self):
    h = TurnIntentHold()
    m = _arm(h)
    assert h.turn_hold_curvature > 0.0
    h.update(-0.05, _cs(v=0.6, right=True), m, True, 0.0)
    assert h.turn_hold_curvature == 0.0 and h.turn_hold_done

  def test_handoff_when_the_action_takes_over(self):
    h = TurnIntentHold()
    m = _arm(h)
    held = h.turn_hold_curvature
    assert held > 0.0
    for _ in range(int(0.4 / DT_CTRL)):
      h.update(held, _cs(v=0.6, right=True), m, True, 0.0)
    assert h.turn_hold_curvature == 0.0 and h.turn_hold_done

  def test_released_above_release_speed(self):
    h = TurnIntentHold()
    m = _arm(h)
    assert h.turn_hold_curvature > 0.0
    h.update(0.01, _cs(v=CURVATURE_HOLD_RELEASE_SPEED + 1.0, right=True), m, True, 0.01)
    assert h.turn_hold_curvature == 0.0, "hold not released above RELEASE_SPEED"
    # The hold is released, but TURN_LEAD is gated on [MIN_SPEED, MAX_SPEED) independently
    # of RELEASE_SPEED, so pass-through only holds where the lead is also out of range.
    out = h.update(0.01, _cs(v=TURN_LEAD_MAX_SPEED + 1.0, right=True), m, True, 0.01)
    assert out == 0.01, "command must pass through untouched with both stages out of range"
    # ...or in range but with no corner in the plan
    out = h.update(0.01, _cs(v=4.0, right=True), _model(curve=0.0), True, 0.01)
    assert out == 0.01

  def test_standstill_timeout_drops_the_hold(self):
    h = TurnIntentHold()
    _arm(h)
    assert h.turn_hold_curvature > 0.0
    # stopped, and the plan no longer shows a corner so nothing re-arms it
    straight = _model(curve=0.0)
    for _ in range(int((CURVATURE_HOLD_STANDSTILL_TIMEOUT + 1.0) / DT_CTRL)):
      h.update(0.0, _cs(v=0.0, right=True), straight, True, 0.0)
    assert h.turn_hold_curvature == 0.0

  def test_hold_survives_lateral_going_inactive(self):
    """Retention must NOT depend on latActive -- lateral drops out at standstill on torque
    cars, which is the exact window this bridges."""
    h = TurnIntentHold()
    m = _arm(h)
    held = h.turn_hold_curvature
    assert held > 0.0
    for _ in range(300):
      h.update(0.0, _cs(v=0.6, right=True), m, False, 0.0)
    assert h.turn_hold_curvature > 0.0, "hold was lost while lateral was inactive"

  def test_blinker_flip_clears_the_hold(self):
    h = TurnIntentHold()
    m = _arm(h)
    assert h.turn_hold_curvature > 0.0
    for _ in range(50):
      h.update(0.0, _cs(v=0.6, left=True), m, True, 0.0)
    assert h.turn_hold_curvature <= 0.0

  def test_plan_source_only_below_its_speed(self):
    """Above PLAN_SOURCE_SPEED only the model's own action may raise the hold, so the
    pre-wind cannot capture a corner while rolling."""
    h = TurnIntentHold()
    m = _model(curve=0.06)
    for _ in range(200):
      h.update(0.0, _cs(v=CURVATURE_HOLD_PLAN_SOURCE_SPEED + 0.5, right=True), m, True, 0.0)
    assert h.turn_hold_curvature == 0.0

  def test_turn_lead_excluded_during_lane_change(self):
    """A lane-change blinker is not turn intent."""
    v = 3.5   # inside [TURN_LEAD_MIN_SPEED, TURN_LEAD_MAX_SPEED)
    normal = TurnIntentHold().update(0.0, _cs(v=v, right=True), _model(curve=0.08), True, 0.0)
    during = TurnIntentHold().update(0.0, _cs(v=v, right=True), _model(curve=0.08, lane_change=True), True, 0.0)
    assert normal > 0.0, "lead did not fire on a normal signalled turn"
    assert during == 0.0, "lead fired during a lane change"

  def test_turn_lead_vetoed_when_braking_to_a_stop(self):
    v = 3.5
    rolling = TurnIntentHold().update(0.0, _cs(v=v, right=True, a=0.0), _model(curve=0.08), True, 0.0)
    braking = TurnIntentHold().update(0.0, _cs(v=v, right=True, a=-2.0), _model(curve=0.08), True, 0.0)
    assert rolling > 0.0
    assert braking == 0.0, "lead fired while braking to a stop short of the arc"

  def test_output_stays_bounded(self):
    """The hold can latch the model's own command, which PLAN_CAP does not bound (that caps
    only the plan-probe source), so the output is bounded by the largest command seen plus
    the probe caps -- never by the probe caps alone."""
    h = TurnIntentHold()
    rng = np.random.default_rng(11)
    cap = max(CURVATURE_HOLD_PLAN_CAP, TURN_LEAD_CAP)
    peak = 0.0
    for _ in range(2000):
      v, cmd, curv = rng.uniform(0, 8), rng.normal(0, 0.05), rng.normal(0, 0.02)
      cmd = float(cmd)
      peak = max(peak, abs(cmd))
      out = h.update(cmd, _cs(v=float(v), right=True), _model(curve=float(rng.normal(0, 0.05))), True, float(curv))
      assert abs(out) <= max(peak, cap) + 1e-9, (cmd, out, peak)

  def test_release_speed_is_10_mph(self):
    assert np.isclose(CURVATURE_HOLD_RELEASE_SPEED, 10.0 * CV.MPH_TO_MS)
