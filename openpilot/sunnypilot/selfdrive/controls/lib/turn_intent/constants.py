"""
Low-speed turn-intent curvature hold, ported from StarPilot's controlsd.py.

Approaching a turn with the blinker on, the model's time-based plan collapses as the car
slows: desiredCurvature decays to zero, the controller unwinds the wheel at the
intersection, and on pull-away it re-winds too late, so the car goes wide. This module
holds a floor on the commanded curvature through that window and hands back to the model
as soon as its own action wakes up.

Kept out of controlsd: the whole feature is one call that takes the model's desired
curvature and returns a possibly-raised one, plus a reset(). All state lives here.

Everything below is blinker-gated -- with no blinker, `update()` returns the input
unchanged, so this cannot affect ordinary lane keeping.
"""
import math

from openpilot.common.constants import CV

# Low-speed turn-intent curvature hold. Approaching a turn with the blinker on, the
# model's time-based plan collapses as the car slows to a stop: desiredCurvature decays
# to zero, the controller actively unwinds the wheel at the intersection, and on
# pull-away it re-winds too late — the car goes wide (pauseturn rlog 2026-07-13).
# The hold ratchets up on the blinker-matching model command below the release speed
# and floors the command magnitude afterwards. Below the hard speed the floor is firm;
# between hard and release speed it decays toward the model's sustained demand, so a
# transient model dip barely sags it while a genuine end-of-turn unwind or an aborted
# turn still drains it in a few seconds. Retention deliberately does NOT depend on the
# blinker (the stalk auto-cancels during the stop in the log), on latActive (lateral
# goes inactive at standstill on torque cars; the wheel parks on rack friction), or on
# steeringPressed (the driver's instinctive grip during the unwind is what let the
# collapse through, and the driver physically overpowers a torque command regardless).
CURVATURE_HOLD_HARD_SPEED = 4.5 * CV.MPH_TO_MS
# Release must sit ABOVE the speed where the model's action wakes up mid-turn. Left
# turns cross the intersection before arcing, so the car reaches ~3.5-4 m/s while the
# action still reads ~0 (left1/left2 rlogs 2026-07-15: a 6 mph release dropped the
# floor mid-turn and visibly unwound the wheel 50 deg before the action woke; rights
# wake at ~2.2 m/s and never showed it). Hold authority above creep speed is bounded
# by the opposite-command release and the decay band, not by this ceiling.
CURVATURE_HOLD_RELEASE_SPEED = 10.0 * CV.MPH_TO_MS
# Pre-wind is a NEAR-STANDSTILL device: winding the wheel is only free when the car
# isn't moving. On rolling slow turns a plan-sourced floor applies the turn's final
# curvature at the entry, starting the arc 4-7 m early — the "turning too much"
# corrections in the stickyright1/2 and left-crossing rlogs (2026-07-16) all trace to
# plan capture while moving. Above this speed only the model's own action can raise
# the hold, rate-limited so a single-frame action spike (left1 rlog 21.9s: -0.157 for
# one model frame) can't get captured and floored for seconds.
CURVATURE_HOLD_PLAN_SOURCE_SPEED = 2.0 * CV.MPH_TO_MS
CURVATURE_HOLD_RATCHET_RATE = 0.04  # 1/m per s, hold growth limit above the plan-source speed
# Once the model's action has sustainably taken over the turn, hand off COMPLETELY:
# clear the hold and don't re-engage until the blinker cycle ends. A floor that chases
# the awake action only distorts the model's entry spiral, mid-turn shape, and exit
# unwind — the sticky right-turn exits were the floor trailing the model's unwind by
# 0.03-0.05 even at the fast decay (stickyright1 41.15s: floor 0.064 vs action 0.014,
# driver correcting +394). The bridge job is done the moment the action is awake.
CURVATURE_HOLD_HANDOFF_FRAC = 0.75
CURVATURE_HOLD_HANDOFF_TIME = 0.3  # s of sustained action >= frac*hold before handoff
CURVATURE_HOLD_DECAY_TAU = 2.0     # s; hold tracks a sustained lower model demand with this time constant
# At a turn exit the model unwinds through small SAME-sign commands (curvature only
# flips negative for the final counter-steer), so the opposite-release fires late and
# the tau-2 decay melts the floor slower than the model's exit ramp — the car keeps
# arcing while the driver hauls the wheel back (tightright3/4 rlogs 2026-07-15, drv
# +500). Turn progress is the discriminator between a mid-turn dip (protect the floor;
# left-turn sags happened at ~20 deg of swept heading) and an exit (drop it; the
# sticks happened past ~80 deg): once the swept heading passes the threshold, the
# decay switches to the fast tau and runs at ANY speed, including below the hard-hold
# speed. Swept resets whenever the hold disengages.
CURVATURE_HOLD_SWEPT_EXIT = 0.9    # rad of heading actually turned (~52 deg)
CURVATURE_HOLD_EXIT_DECAY_TAU = 0.5  # s
CURVATURE_HOLD_STANDSTILL_TIMEOUT = 30.0  # s stopped before the held turn intent is dropped

# The model's time-domain action.desiredCurvature is blind below ~2.5 m/s (0.3 s ahead
# at creep speed is centimeters of road), but the plan's spatial geometry already shows
# the turn at standstill: turn3/turn4 rlogs 2026-07-14 read plan curvature 0.13-0.16
# while the action output sat at 0.005, and the plan value matched the demand the
# action produced once rolling. Feeding it into the turn-hold ratchet lets the wheel
# pre-wind toward the real turn before the car moves. Scaled and capped conservatively:
# a too-high floor turns in tighter than the path (mild at creep lat accel), while a
# too-low one just reduces the head start. The plan flickers straight for ~1-2 s right
# at the standstill->motion transition; the ratchet holds through it by design.
CURVATURE_HOLD_PLAN_LOOKAHEAD_NEAR = 4.0  # m; reads whether the turn starts NOW
CURVATURE_HOLD_PLAN_LOOKAHEAD_FAR = 7.0   # m; reads the turn's curvature
CURVATURE_HOLD_PLAN_SCALE = 0.85
CURVATURE_HOLD_PLAN_CAP = 0.12       # 1/m
# Proximity gate on the pre-wind: the 4/7 m probe reads the corner's full curvature the
# instant the plan bends toward it, which at a stop-line turn is several meters before
# the car reaches the line — so it wound the wheel hard while still rolling and cut the
# inside curb (both directions), and on an early blinker it turned in before the corner
# (RL2 seg1 t=42: corner 2 m ahead, car at 1.2 m/s, pre-wind already at 0.13). The plan
# itself carries the distance: get_plan_turn_onset_dist is where the path bends. Scale
# the pre-wind from full at ONSET_NEAR to zero at ONSET_FAR_GATE so it winds only as the
# corner closes. At a real stop the onset collapses to ~0 m once stopped (model: "turn
# is here"), so the stop-line pre-wind still reaches full strength — just later, at the
# line instead of 3 m short of it.
CURVATURE_HOLD_ONSET_HEADING = 10.0    # deg of plan heading change that marks the corner
CURVATURE_HOLD_ONSET_NEAR = 1.5        # m; corner this close -> full pre-wind
CURVATURE_HOLD_ONSET_FAR_GATE = 5.0    # m; corner past this -> no pre-wind yet
CURVATURE_HOLD_ONSET_FAR = 100.0       # m; sentinel "no corner found"
CURVATURE_HOLD_REACH_MIN = 7.0
CURVATURE_HOLD_REACH_FULL = 12.0
# The model counter-steers at every turn exit; an opposite-direction command is the
# "turn is over" signal at any speed. Without this the floor converted the exit unwind
# (-0.076) into a stuck +0.012 for 1.4 s and the driver had to unwind by hand
# (rightturnfail rlog 33.2-34.4s). Deadband rejects the ~0.002 pull-away flickers.
CURVATURE_HOLD_OPPOSITE_RELEASE = 0.01  # 1/m
# Nudge-to-commit: creeping toward a rolling left below the release speed, the model
# often does not commit until the geometry is entered — the driver hauled the wheel
# 144-260 deg alone with the action at zero (0000087c segs 7/8, +377..+411 driver
# torque) before the model took over. Forcing the plan probe here instead is what
# caused the 2026-07-19 fights, and the driver's own torque separates those cases
# perfectly: they RESISTED the premature machine wind (-117, bracing) but PUSH when
# the gap is theirs. So the driver's matching push is the "go" signal: capture the
# curvature they have physically wound into the hold, un-rate-limited (the wheel is
# already there; holding adds no motion), so the car grabs the turn at a nudge instead
# of making them wind it all by hand. Allowed anywhere the hold exists (< release
# speed): a 3 m/s ceiling left the 0000087f seg 1 gap-taken-at-3.5-4.2 m/s haul
# (40->220 deg at +418) unassisted; the decay/handoff/opposite-release machinery
# already bounds the captured floor in that band.
CURVATURE_HOLD_CONFIRM_MIN = 0.003  # 1/m (~7 deg) of wound curvature before capture
CURVATURE_HOLD_CONFIRM_SWEPT = 0.6  # rad of heading swept this blinker cycle; past this the push is exit-shaping, not initiation


def _plan_circle_curvature(xs, ys, lookahead: float) -> float:
  # curvature of the circle through the origin, tangent to the car's heading, passing
  # through the plan point ~lookahead meters ahead: kappa = 2y / (x^2 + y^2)
  px, py = 0.0, 0.0
  for x, y in zip(xs, ys, strict=False):
    px, py = x, y
    if math.hypot(x, y) >= lookahead:
      break
  d2 = px * px + py * py
  if d2 < 1.0:
    return 0.0
  return 2.0 * py / d2


def _plan_dual_probe(model_v2, d_near: float, d_far: float) -> float:
  # Min-magnitude of a near and a far circle fit. The far probe alone assumes the turn
  # starts immediately, which over-winds wide turns whose arc begins several meters out
  # (wide multi-lane lefts): the near probe reads ~straight there and only grows as the
  # car approaches the arc, so the readout self-scales to the turn geometry. Sign
  # disagreement means no coherent turn ahead: contribute nothing.
  xs, ys = model_v2.position.x, model_v2.position.y
  near = _plan_circle_curvature(xs, ys, d_near)
  far = _plan_circle_curvature(xs, ys, d_far)
  if near * far <= 0.0:
    return 0.0
  return near if abs(near) < abs(far) else far


def get_plan_spatial_curvature(model_v2) -> float:
  return _plan_dual_probe(model_v2, CURVATURE_HOLD_PLAN_LOOKAHEAD_NEAR, CURVATURE_HOLD_PLAN_LOOKAHEAD_FAR)


def get_plan_turn_onset_dist(model_v2) -> float:
  # Distance along the plan at which the path first bends past ONSET_HEADING from the
  # car's current heading — i.e. how far ahead the corner actually starts. The pre-wind
  # magnitude scales on this so a stop-line turn winds only once the corner is close,
  # not the instant the plan first shows a turn several meters out (which cut the inside
  # curb at a stop and turned in early on an over-eager blinker). Returns a large
  # sentinel when no bend is found, so distant/straight plans read "far" and don't wind.
  xs, ys = model_v2.position.x, model_v2.position.y
  n = min(len(xs), len(ys))
  for i in range(2, n):
    dx = xs[i] - xs[i - 1]
    dy = ys[i] - ys[i - 1]
    if abs(dx) < 1e-3 and abs(dy) < 1e-3:
      continue
    if abs(math.degrees(math.atan2(dy, dx))) > CURVATURE_HOLD_ONSET_HEADING:
      return math.hypot(xs[i], ys[i])
  return CURVATURE_HOLD_ONSET_FAR


def get_plan_reach(model_v2) -> float:
  xs = model_v2.position.x
  return xs[-1] if len(xs) else 0.0


# Turn-initiation lead. The model's action and the fixed 4/7 m probes are anchored in
# METERS, so the seconds of warning they give shrinks with speed — at 12 mph a corner
# enters the 7 m window only ~1.3 s out, too late to wind the wheel, which is why every
# turn in the Desktop/new drive initiated only below ~5.5 m/s regardless of approach
# speed. This lead probes the plan at a constant-TIME distance instead and max-mag
# blends into the command, so initiation can start while still rolling at 9-12 mph.
# The engagement fade (authority ramps to zero as measured curvature approaches the
# lead) limits it to the initiation phase: once the car is tracking the arc, the model
# owns the turn shape — without it, the blend re-creates the 2026-07-16 rolling-turn
# front-load (stickyright2 entry: 0.084 commanded vs the model's own 0.041 spiral). A
# binary gate here limit-cycles: cutting drops demand to a still-small action, the
# wheel unwinds below the threshold, the lead re-fires — a 5 Hz demand sawtooth felt
# as wiggle (2026-07-19 drive seg 7 t=49-50). The fade instead settles at a stable
# ~2/3-of-lead equilibrium until the action takes over via the max-mag blend.
# Stop-sign approaches stay quiet because the stopping plan compresses to the stop
# line and reads ~straight until the car is nearly there (probes ~0 for 7 s of
# blinker-on coasting at 9-12 m/s). Below TURN_LEAD_MIN_SPEED the lead must stay OFF,
# not just for standstill plan flicker: at creep speed the probe's 4 m distance floor
# chord-fits a left turn's straight-then-arc entry as "arc now", demanding 3-5x the
# model's intent. The model fights back — its plan flips to correct the premature yaw,
# the wheel swings back through center, and the swing trips the stalk auto-cancel,
# killing the turn (2026-07-19 segs 6/7/8/10, all at 1.5-2.4 m/s; the same drive's
# completed turns show the model committing on its own below 3 m/s). The model's
# meter-anchored horizon gives it 2+ s of warning at creep speed — the lead's whole
# reason to exist (time-warning shrinking with speed) does not apply there.
TURN_LEAD_T = 1.3          # s of travel the probe looks ahead (~wind-up time + lat delay)
TURN_LEAD_MIN_M = 4.0
TURN_LEAD_MAX_M = 14.0
TURN_LEAD_MIN_SPEED = 3.0   # m/s: authority 0 here, ramps to full at FULL_SPEED
TURN_LEAD_FULL_SPEED = 4.0  # m/s
TURN_LEAD_MAX_SPEED = 7.0   # m/s (~15.7 mph)
TURN_LEAD_SCALE = 0.85
TURN_LEAD_CAP = 0.12       # 1/m
TURN_LEAD_ENGAGED_FRAC = 0.5   # engagement fade starts here, zero authority at 1.0
TURN_LEAD_MODEL_OPPOSE = 0.003  # 1/m: model steering this hard against the blinker vetoes the lead
# Braking-to-a-stop veto: if sustaining the current decel parks the car within this
# factor of the probe distance, the driver intends to stop short of the arc — demanding
# that arc's curvature NOW winds the wheel at a stop approach the model didn't plan
# (0000087c seg 1: lead wound 30 deg at 6 m/s while braking for a 13 s stop; driver
# yanked it back with -509 torque). Turn-approach braking releases before this trips;
# a held brake to standstill keeps it tripped, deferring the turn to the pre-wind.
TURN_LEAD_STOP_MARGIN = 1.5
TURN_LEAD_DECEL_GATE = -0.5  # m/s^2: only project a stop when genuinely braking
