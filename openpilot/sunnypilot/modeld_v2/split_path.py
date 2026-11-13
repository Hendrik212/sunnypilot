"""Split path for an action-head model (LatSplitPathHz param, Hz, default 0 = off).

The lateral-acceleration request of the model is the sum of two things: its reaction to the car's
own small motion in the lane (the lane-keeping loop: model -> car -> camera -> model) and its
response to the scene. Any filter on the whole request delays the loop, which has 13-21 deg of
phase margin at 0.30 Hz, and the 0.3 Hz weave comes back; that is the weave/chatter trade-off of
every lever tried before (notch Q, look-ahead, LatSmoothSeconds, kp).

Replay sim, 2026-11-13, Cheddar on routes 0000026a segs 14-16 and 0000026d segs 21-24:
- Open loop with the recorded car's own 0.15-3 Hz motion removed from the view, the request drops
  60-67% at 0.25-0.45 Hz and 41-45% at 0.5-1 Hz: the chatter is about half reaction, half scene.
- The reaction is predictable from the gyro: a linear fit on the high-passed yaw rate and its two
  leaky integrals (heading, lateral offset) explains 83-91% at 0.5-1 Hz on segments of both roads
  the fit never saw (gain 0.61-0.74, phase -7..-17 deg). Below ~0.2 Hz it is NOT predictable.

So: request = low_pass(request - reaction_estimate) + reaction_estimate. The loop keeps its phase,
only the scene part is filtered. Closed loop against the car's setting, with recorded gyro noise,
first-order 0.5 Hz and the gate below: weave -15..0%, chatter 0.5-1 Hz x0.63-0.83, 1-2 Hz
x0.50-0.66, torque rate p95 x0.68-0.72, lane wobble 2.8-3.3 cm (2.4-3.1 without). The same low-pass
on the whole request is unstable. Second order or a lower cutoff brings a slow 0.1-0.18 Hz lane
mode (5-9 cm), a sharp "remove only above fc" form raises the weave by 24-165%.

Limits that are built in, not tunable:
- Reaction gain. The loop has 2-3 dB of gain margin; an estimate 60% too high went unstable in one
  of four runs, 30% either way was stable. The fit under-predicts, which is the safe side.
- Gate. The gyro cannot tell a deliberate turn-in from wiggle: on the logged requests of routes
  00000275/276 the ungated split reached 0.7 m/s^2 650-1000 ms late (10 events). The split fades
  out as the request magnitude rises from GATE_BP[0] to GATE_BP[1] and returns over ~1 s.
- Highway only (SPEED_BP): the sim covers 90-130 km/h, and bends are the risk.
- Cheddar only: the weights are a fit of that model's reaction (modeld enables it by pkl name).
"""
import numpy as np

DT = 0.05  # s, model rate
SPLIT_HZ_MIN, SPLIT_HZ_MAX = 0.35, 0.6  # scene low-pass cutoff range; below MIN the slow lane mode grows
SPEED_BP = [20.0, 25.0]  # m/s: off below 72 km/h, full from 90 km/h
GATE_BP = [0.3, 0.6]  # m/s^2 of request magnitude: full split below, none above
GATE_RELEASE_S = 1.0
FEATURE_HP_HZ = 0.1
MIN_SPEED = 1.0
# reaction [m/s^2] = W_E . (e[n], e[n-1]) + W_PSI . (psi[n], psi[n-1]) + W_A . (a[n], a[n-1])
# e [m], psi [rad]: leaky integrals of the high-passed curvature; a = v^2 * high-passed curvature [m/s^2]
W_E = (-0.66312494, -0.51854633)
W_PSI = (-15.19588269, -17.13200252)
W_A = (0.19653271, -0.35152769)


def _k(fc: float) -> float:
  return float(np.tan(np.pi * fc * DT))


class _HighPass1:
  """First-order Butterworth high-pass (bilinear), starts at rest."""
  def __init__(self, fc: float):
    k = _k(fc)
    self.b, self.a1 = 1.0 / (1.0 + k), (k - 1.0) / (k + 1.0)
    self.x1 = self.y1 = 0.0

  def __call__(self, x: float) -> float:
    y = self.b * (x - self.x1) - self.a1 * self.y1
    self.x1, self.y1 = x, y
    return y


class SplitPath:
  def __init__(self):
    self.reset()

  def reset(self) -> None:
    self.hp_k, self.hp_psi, self.hp_e = _HighPass1(FEATURE_HP_HZ), _HighPass1(FEATURE_HP_HZ), _HighPass1(FEATURE_HP_HZ)
    self.k0: float | None = None
    self.int_psi = self.int_e = 0.0
    self.e = [0.0, 0.0]
    self.psi = [0.0, 0.0]
    self.acc = [0.0, 0.0]
    self.lp_x1: float | None = None  # low-pass state: previous input and output
    self.lp_y1 = 0.0
    self.gate_peak = 0.0
    self.reaction = 0.0  # last predicted reaction (m/s^2), for the log
    self.weight = 0.0  # last applied share of the split (0..1), for the log

  def _reaction(self, yaw_rate: float, v_ego: float) -> float:
    v = max(v_ego, MIN_SPEED)
    k = yaw_rate / v
    if self.k0 is None:
      self.k0 = k
    kc = self.hp_k(k - self.k0)
    self.int_psi += v * kc * DT
    psi = self.hp_psi(self.int_psi)
    self.int_e += v * psi * DT
    e = self.hp_e(self.int_e)
    self.e = [e, self.e[0]]
    self.psi = [psi, self.psi[0]]
    self.acc = [kc * v * v, self.acc[0]]
    return (W_E[0] * self.e[0] + W_E[1] * self.e[1] + W_PSI[0] * self.psi[0] + W_PSI[1] * self.psi[1] +
            W_A[0] * self.acc[0] + W_A[1] * self.acc[1])

  def update(self, lat_accel: float, yaw_rate_measured: float | None, v_ego: float, split_hz: float) -> float:
    """lat_accel with the scene part low-passed at split_hz. Call once per model frame, also while
    the split is off, so that it is settled when a live param switches it on."""
    if yaw_rate_measured is None or not np.isfinite(yaw_rate_measured) or not np.isfinite(lat_accel):
      self.reset()
      return lat_accel
    self.reaction = self._reaction(float(yaw_rate_measured), v_ego)
    scene = lat_accel - self.reaction

    self.gate_peak = max(abs(lat_accel), self.gate_peak * float(np.exp(-DT / GATE_RELEASE_S)))
    on = split_hz > 0.0 and np.isfinite(split_hz)
    self.weight = (float(np.interp(v_ego, SPEED_BP, [0.0, 1.0])) * float(np.interp(self.gate_peak, GATE_BP, [1.0, 0.0]))) if on else 0.0

    if not on or self.lp_x1 is None:
      # track the scene part: switching on starts from "nothing removed", without a step
      self.lp_x1 = self.lp_y1 = scene
      return lat_accel
    k = _k(min(max(split_hz, SPLIT_HZ_MIN), SPLIT_HZ_MAX))
    scene_lp = (k * (scene + self.lp_x1) - (k - 1.0) * self.lp_y1) / (k + 1.0)
    self.lp_x1, self.lp_y1 = scene, scene_lp
    return lat_accel - self.weight * (scene - scene_lp)
