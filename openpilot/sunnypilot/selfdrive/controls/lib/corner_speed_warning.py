"""Corner speed warning (LatCornerSpeedWarnAccel param, m/s^2, default 0 = off).

The Ioniq 6 EPS caps the LKAS torque at about 450 CAN (eps limit test, Nov 7). Routes 00000275-0000027d,
hands light, steady wheel angle, output at the cap: the car delivers 2.8-3.1 m/s^2 of lateral acceleration
at 25-55 km/h. The driver sets the speed (no openpilot longitudinal), and corners under R 40 m were taken at
2.9-5.0 m/s^2 (median 4.1): the car turns 65-79 % of the request until the driver helps. The model's plan
is right there (peak request 0.9-1.04 of the driven curvature for R 25-120 m).

So: warn before the corner. previewed = max curvature of the model plan over the next T_MAX seconds times
the current speed squared; alert while previewed >= the param. The text gives the speed the steering can
hold: sqrt(limit / curvature).

Replay of this module on those nine routes (2.3 h engaged, 45 events with the output at the cap), limit 2.8:
39 of the 45 cap events warned (87 %), lead 2.4 s median and 0.6 s for the worst tenth; 38 alerts per engaged
hour on these roads, lasting 5.3 s median: 51 of 87 before a cap event, 35 before a hard corner the steering
did manage, 1 with no hard corner. Limit 2.4: 91 % and 46 per hour; 3.2: 76 % and 29 per hour. A 2 s window
gives 1.4 s of lead, 4 s more alerts without a hard corner. Crediting the current deceleration lowered the
share warned to 69 %: left out, the alert ends once the car is slow enough.
The plan speed is |velocity| (x and y): velocity.x alone reads up to 27 % off in tight corners; with the
norm the log fields match the replay's path-length speed within 2 %.
Not covered: hairpins under R 25 m, where the plan itself asks 0.6-0.85 of the corner. Not driven yet.
"""
import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD

T_MIN, T_MAX = 0.3, 3.0  # s, window of the model plan
MIN_SPEED = 5.0  # m/s: below this, turns at junctions
LIMIT_MIN, LIMIT_MAX = 2.0, 4.0  # m/s^2, range of the param when it is on
ON_FRAMES = 3  # consecutive model frames above the limit
OFF_RATIO = 0.9  # the alert ends when previewed stays below this share of the limit ...
OFF_S = 1.0  # ... for this long
MIN_PLAN_SPEED = 1.0  # m/s


def preview_curvature(t: np.ndarray, yaw_rate: np.ndarray, v_plan: np.ndarray) -> float:
  """Largest curvature (1/m) of the plan between T_MIN and T_MAX."""
  # arrays first: a capnp list cannot be sliced
  t, yaw_rate, v_plan = np.asarray(t, dtype=float), np.asarray(yaw_rate, dtype=float), np.asarray(v_plan, dtype=float)
  n = min(len(t), len(yaw_rate), len(v_plan))
  if n < 2:
    return 0.0
  t, yaw_rate, v_plan = t[:n], yaw_rate[:n], v_plan[:n]
  window = (t >= T_MIN) & (t <= T_MAX)
  if not window.any():
    return 0.0
  curvature = np.abs(yaw_rate[window]) / np.maximum(v_plan[window], MIN_PLAN_SPEED)
  peak = float(np.max(curvature))
  return peak if np.isfinite(peak) else 0.0


class CornerSpeedWarning:
  def __init__(self):
    self.params = Params()
    self.frame = -1
    self.limit = self._read_limit()
    self.active = False
    self.preview_lat_accel = 0.0  # m/s^2, for the log
    self.safe_speed = 0.0  # m/s, lowest speed asked during the current alert
    self._on_frames = 0
    self._off_frames = 0

  def _read_limit(self) -> float:
    try:
      limit = float(self.params.get("LatCornerSpeedWarnAccel", return_default=True))
    except (TypeError, ValueError):
      return 0.0
    if not np.isfinite(limit) or limit <= 0.0:
      return 0.0
    return float(np.clip(limit, LIMIT_MIN, LIMIT_MAX))

  def step(self, curvature: float, v_ego: float, allowed: bool, limit: float) -> bool:
    """One model frame. curvature = preview_curvature(...), allowed = lateral control on, no blinker."""
    self.preview_lat_accel = curvature * v_ego * v_ego
    if limit <= 0.0 or not allowed or v_ego < MIN_SPEED:
      self.active = False
      self._on_frames = self._off_frames = 0
      self.safe_speed = 0.0
      return False

    self._on_frames = self._on_frames + 1 if self.preview_lat_accel >= limit else 0
    self._off_frames = self._off_frames + 1 if self.preview_lat_accel < OFF_RATIO * limit else 0
    if not self.active and self._on_frames >= ON_FRAMES:
      self.active = True
      self.safe_speed = v_ego
    elif self.active and self._off_frames * DT_MDL >= OFF_S:
      self.active = False
      self.safe_speed = 0.0
    if self.active and self.preview_lat_accel >= limit:
      self.safe_speed = min(self.safe_speed, float(np.sqrt(limit / max(curvature, 1e-6))))
    return self.active

  def update(self, sm: messaging.SubMaster, events_sp) -> None:
    self.frame += 1
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.limit = self._read_limit()

    model = sm['modelV2']
    CS = sm['carState']
    # planned speed along the path: in a tight corner the path leaves the x axis, so velocity.x alone reads low
    vx, vy = np.asarray(model.velocity.x, dtype=float), np.asarray(model.velocity.y, dtype=float)
    v_plan = np.hypot(vx, vy) if len(vx) == len(vy) else np.zeros(0)
    curvature = preview_curvature(model.orientationRate.t, model.orientationRate.z, v_plan)
    allowed = sm['carControl'].latActive and not (CS.leftBlinker or CS.rightBlinker)
    if self.step(curvature, CS.vEgo, allowed, self.limit):
      events_sp.add(custom.OnroadEventSP.EventName.cornerSpeed)
