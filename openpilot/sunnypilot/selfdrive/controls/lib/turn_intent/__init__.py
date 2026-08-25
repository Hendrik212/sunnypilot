"""
Low-speed turn-intent curvature hold.

Public surface is one object:

    self.turn_intent = TurnIntentHold()
    ...
    new_desired_curvature = self.turn_intent.update(
      new_desired_curvature, CS, model_v2, lat_active, measured_curvature)

Everything is blinker-gated: with no blinker the input is returned unchanged, so this
cannot affect ordinary lane keeping.

NOTE: `reset()` exists for construction only -- do NOT call it when lateral goes inactive.
Retention deliberately does not depend on latActive: lateral goes inactive at standstill on
torque cars and the wheel parks on rack friction, which is exactly the window the hold is
there to bridge. The state machine self-clears above CURVATURE_HOLD_RELEASE_SPEED, on an
opposite-direction model command, on handoff, and after
CURVATURE_HOLD_STANDSTILL_TIMEOUT stopped.

Ported from StarPilot's controlsd.py; the state machine below is a transcription of the
block that ran inline there, with `self.<x>` renamed off the Controls object onto this one.
See constants.py for the constants and the plan probes, both copied verbatim.
"""
import math

from openpilot.cereal import log
from openpilot.common.realtime import DT_CTRL

from openpilot.sunnypilot.selfdrive.controls.lib.turn_intent.constants import (
  CURVATURE_HOLD_CONFIRM_MIN,
  CURVATURE_HOLD_CONFIRM_SWEPT,
  CURVATURE_HOLD_DECAY_TAU,
  CURVATURE_HOLD_EXIT_DECAY_TAU,
  CURVATURE_HOLD_HANDOFF_FRAC,
  CURVATURE_HOLD_HANDOFF_TIME,
  CURVATURE_HOLD_HARD_SPEED,
  CURVATURE_HOLD_ONSET_FAR_GATE,
  CURVATURE_HOLD_ONSET_NEAR,
  CURVATURE_HOLD_OPPOSITE_RELEASE,
  CURVATURE_HOLD_PLAN_CAP,
  CURVATURE_HOLD_PLAN_SCALE,
  CURVATURE_HOLD_PLAN_SOURCE_SPEED,
  CURVATURE_HOLD_RATCHET_RATE,
  CURVATURE_HOLD_REACH_FULL,
  CURVATURE_HOLD_REACH_MIN,
  CURVATURE_HOLD_RELEASE_SPEED,
  CURVATURE_HOLD_STANDSTILL_TIMEOUT,
  CURVATURE_HOLD_SWEPT_EXIT,
  TURN_LEAD_CAP,
  TURN_LEAD_DECEL_GATE,
  TURN_LEAD_ENGAGED_FRAC,
  TURN_LEAD_FULL_SPEED,
  TURN_LEAD_MAX_M,
  TURN_LEAD_MAX_SPEED,
  TURN_LEAD_MIN_M,
  TURN_LEAD_MIN_SPEED,
  TURN_LEAD_MODEL_OPPOSE,
  TURN_LEAD_SCALE,
  TURN_LEAD_STOP_MARGIN,
  TURN_LEAD_T,
  _plan_dual_probe,
  get_plan_reach,
  get_plan_spatial_curvature,
  get_plan_turn_onset_dist,
)

LaneChangeState = log.LaneChangeState


class TurnIntentHold:
  def __init__(self):
    self.reset()

  def reset(self):
    self.turn_hold_curvature = 0.0
    self.turn_hold_standstill_t = 0.0
    self.turn_hold_swept = 0.0
    self.turn_hold_handoff_t = 0.0
    self.turn_hold_done = False
    self.turn_blinker_swept = 0.0

  def update(self, new_desired_curvature, CS, model_v2, lat_active, curvature):
    """Return the commanded curvature, raised to the turn-intent floor where one is held.

    `curvature` is the car's MEASURED curvature (controlsd's self.curvature).
    """
    # Curvature sign convention here is positive for RIGHT turns (a left turn at +148 deg
    # steering angle logs desiredCurvature -0.07), so the blinker maps right=+1, left=-1.
    blinker_dir = float(CS.rightBlinker) - float(CS.leftBlinker)
    # heading swept in the blinker's direction over the whole blinker cycle (any speed):
    # discriminates a turn not yet made from one being exited (see the re-arm below)
    if blinker_dir == 0.0:
      self.turn_blinker_swept = 0.0
    else:
      self.turn_blinker_swept += max(CS.vEgo * curvature * blinker_dir, 0.0) * DT_CTRL

    if CS.vEgo >= CURVATURE_HOLD_RELEASE_SPEED:
      self.turn_hold_curvature = 0.0
      self.turn_hold_standstill_t = 0.0
      self.turn_hold_swept = 0.0
      self.turn_hold_handoff_t = 0.0
      self.turn_hold_done = False
    else:
      if self.turn_hold_curvature == 0.0:
        self.turn_hold_swept = 0.0
      else:
        # heading actually swept in the hold's direction: the measure of turn progress
        self.turn_hold_swept += max(CS.vEgo * curvature * math.copysign(1.0, self.turn_hold_curvature), 0.0) * DT_CTRL
      turn_exiting = self.turn_hold_swept > CURVATURE_HOLD_SWEPT_EXIT
      if (CS.vEgo > CURVATURE_HOLD_HARD_SPEED or turn_exiting) and lat_active and self.turn_hold_curvature != 0.0:
        # Decay toward the model's sustained same-direction demand instead of leaking on
        # wall-clock time: a wall-clock leak drained the floor mid-turn while the model
        # dipped transiently, while sustained low demand (end of turn, abort) still drains
        # the hold within a couple of time constants.
        hold_dir = math.copysign(1.0, self.turn_hold_curvature)
        model_mag = max(new_desired_curvature * hold_dir, 0.0)
        if model_mag < abs(self.turn_hold_curvature):
          decay_tau = CURVATURE_HOLD_EXIT_DECAY_TAU if turn_exiting else CURVATURE_HOLD_DECAY_TAU
          decayed = abs(self.turn_hold_curvature) + (model_mag - abs(self.turn_hold_curvature)) * (DT_CTRL / decay_tau)
          self.turn_hold_curvature = math.copysign(decayed, self.turn_hold_curvature)
      if CS.vEgo < 0.5:
        self.turn_hold_standstill_t += DT_CTRL
        if self.turn_hold_standstill_t > CURVATURE_HOLD_STANDSTILL_TIMEOUT:
          self.turn_hold_curvature = 0.0
        # a stop resets the turn cycle: the model goes blind again, so a prior handoff
        # must not block the standstill pre-wind
        self.turn_hold_done = False
      else:
        self.turn_hold_standstill_t = 0.0
      if lat_active and self.turn_hold_curvature != 0.0 and \
         new_desired_curvature * math.copysign(1.0, self.turn_hold_curvature) < -CURVATURE_HOLD_OPPOSITE_RELEASE:
        # model is actively counter-steering: the turn is over, release at any speed
        self.turn_hold_curvature = 0.0
        self.turn_hold_done = True
      if lat_active and self.turn_hold_curvature != 0.0 and \
         new_desired_curvature * math.copysign(1.0, self.turn_hold_curvature) >= CURVATURE_HOLD_HANDOFF_FRAC * abs(self.turn_hold_curvature):
        self.turn_hold_handoff_t += DT_CTRL
        if self.turn_hold_handoff_t > CURVATURE_HOLD_HANDOFF_TIME:
          # action has sustainably taken over: hand off completely (see HANDOFF consts)
          self.turn_hold_curvature = 0.0
          self.turn_hold_done = True
      else:
        self.turn_hold_handoff_t = 0.0
      if blinker_dir == 0.0:
        # blinker cycle over: a fresh turn may engage a fresh hold
        self.turn_hold_done = False
      elif lat_active and CS.steeringPressed and CS.steeringTorque * blinker_dir < 0.0 and \
           curvature * blinker_dir > CURVATURE_HOLD_CONFIRM_MIN and \
           self.turn_blinker_swept < CURVATURE_HOLD_CONFIRM_SWEPT:
        # an active driver push into the signaled turn BEFORE the turn is made is fresh
        # turn intent: re-arm the cycle even after a prior handoff. A long blinker-on
        # approach can latch done on a trivial micro-handoff and lock out nudge-to-commit
        # ten seconds later at the real turn. The swept gate keeps a light same-direction
        # touch during the EXIT unwind from re-latching a large hold against the model's
        # recentering.
        self.turn_hold_done = False
      if blinker_dir != 0.0 and not self.turn_hold_done:
        # Ratchet up on the raw model command, never on the floored/measured value, so the
        # hold can't feed itself and defeat the decay. Below the release speed the plan's
        # spatial curvature is the second, earlier-seeing source: it shows the turn at
        # standstill while the action is still blind, letting the pre-wind start before the
        # car moves.
        turn_candidate = new_desired_curvature if lat_active else 0.0
        if lat_active and CS.vEgo < CURVATURE_HOLD_PLAN_SOURCE_SPEED:
          plan_curvature = get_plan_spatial_curvature(model_v2) * CURVATURE_HOLD_PLAN_SCALE
          plan_curvature = max(min(plan_curvature, CURVATURE_HOLD_PLAN_CAP), -CURVATURE_HOLD_PLAN_CAP)
          # Proximity gate (see CURVATURE_HOLD_ONSET_*): wind only as the corner closes, so
          # a stop-line turn winds at the line and an early blinker doesn't turn in early.
          # Full at ONSET_NEAR, zero at ONSET_FAR_GATE.
          onset = get_plan_turn_onset_dist(model_v2)
          onset_w = min(max((CURVATURE_HOLD_ONSET_FAR_GATE - onset) /
                            (CURVATURE_HOLD_ONSET_FAR_GATE - CURVATURE_HOLD_ONSET_NEAR), 0.0), 1.0)
          reach = get_plan_reach(model_v2)
          reach_w = min(max((reach - CURVATURE_HOLD_REACH_MIN) /
                            (CURVATURE_HOLD_REACH_FULL - CURVATURE_HOLD_REACH_MIN), 0.0), 1.0)
          plan_curvature *= onset_w * reach_w
          if plan_curvature * blinker_dir > turn_candidate * blinker_dir:
            turn_candidate = plan_curvature
        # Nudge-to-commit (see CURVATURE_HOLD_CONFIRM_*): the driver actively pushing in the
        # blinker direction at creep speed captures what they have wound. Positive
        # steeringTorque is a LEFT push (negative curvature), so agreement is a negative
        # product with blinker_dir. Exempt from the ratchet rate limit: latching the wheel's
        # current position commands no motion, only keeps the driver's progress.
        driver_confirmed = False
        if lat_active and CS.steeringPressed and \
           CS.steeringTorque * blinker_dir < 0.0 and curvature * blinker_dir > CURVATURE_HOLD_CONFIRM_MIN:
          wound_curvature = max(min(curvature, CURVATURE_HOLD_PLAN_CAP), -CURVATURE_HOLD_PLAN_CAP)
          if wound_curvature * blinker_dir > turn_candidate * blinker_dir:
            turn_candidate = wound_curvature
            driver_confirmed = True
        if turn_candidate * blinker_dir > abs(self.turn_hold_curvature):
          new_mag = turn_candidate * blinker_dir
          if CS.vEgo > CURVATURE_HOLD_PLAN_SOURCE_SPEED and not driver_confirmed:
            new_mag = min(new_mag, abs(self.turn_hold_curvature) + CURVATURE_HOLD_RATCHET_RATE * DT_CTRL)
          self.turn_hold_curvature = math.copysign(new_mag, turn_candidate)
        elif self.turn_hold_curvature * blinker_dir < 0.0:
          # blinker flipped to the other side: turn intent changed
          self.turn_hold_curvature = 0.0
      if lat_active and self.turn_hold_curvature != 0.0:
        hold_dir = math.copysign(1.0, self.turn_hold_curvature)
        if new_desired_curvature * hold_dir < abs(self.turn_hold_curvature):
          new_desired_curvature = self.turn_hold_curvature

    # Turn-initiation lead (see TURN_LEAD_*). Applied AFTER the hold block so the
    # ratchet/handoff only ever see the raw model action; pure max-magnitude, so it can
    # never reduce or oppose the model. Lane changes are excluded: that blinker's plan bend
    # is not a turn. The model-oppose veto is defense-in-depth for the fade-in edge: a model
    # actively steering against the blinker is correcting something the lead must not fight.
    if (lat_active and blinker_dir != 0.0 and
        model_v2.meta.laneChangeState == LaneChangeState.off and
        TURN_LEAD_MIN_SPEED <= CS.vEgo < TURN_LEAD_MAX_SPEED and
        new_desired_curvature * blinker_dir > -TURN_LEAD_MODEL_OPPOSE):
      d_near = max(min(TURN_LEAD_T * CS.vEgo, TURN_LEAD_MAX_M), TURN_LEAD_MIN_M)
      stopping_short = CS.aEgo < TURN_LEAD_DECEL_GATE and \
          CS.vEgo ** 2 / (2.0 * -CS.aEgo) < TURN_LEAD_STOP_MARGIN * d_near
      lead_curvature = 0.0 if stopping_short else _plan_dual_probe(model_v2, d_near, d_near + 3.0) * TURN_LEAD_SCALE
      lead_curvature = max(min(lead_curvature, TURN_LEAD_CAP), -TURN_LEAD_CAP)
      if lead_curvature * blinker_dir > 0.0:
        speed_w = min(max((CS.vEgo - TURN_LEAD_MIN_SPEED) / (TURN_LEAD_FULL_SPEED - TURN_LEAD_MIN_SPEED), 0.0), 1.0)
        engaged_ratio = abs(curvature) / abs(lead_curvature)
        engage_w = min(max((1.0 - engaged_ratio) / (1.0 - TURN_LEAD_ENGAGED_FRAC), 0.0), 1.0)
        lead_curvature *= speed_w * engage_w
        if lead_curvature * blinker_dir > max(new_desired_curvature * blinker_dir, 0.0):
          new_desired_curvature = lead_curvature
          # Capture the applied lead into the hold (rate-limited like any moving-speed
          # ratchet) so decelerating through the fade floor keeps the initiation progress:
          # without this, braking mid-wind dumped the lead's demand back to the still-small
          # action and visibly unwound the wheel before the standstill pre-wind had to redo
          # the work.
          if CS.vEgo < CURVATURE_HOLD_RELEASE_SPEED and not self.turn_hold_done and \
             lead_curvature * blinker_dir > abs(self.turn_hold_curvature):
            held_mag = min(lead_curvature * blinker_dir, abs(self.turn_hold_curvature) + CURVATURE_HOLD_RATCHET_RATE * DT_CTRL)
            self.turn_hold_curvature = math.copysign(held_mag, lead_curvature)

    return new_desired_curvature
