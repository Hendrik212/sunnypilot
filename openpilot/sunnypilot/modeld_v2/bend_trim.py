"""Bend trim: hold the car nearer the lane centre in bends (LatBendTrimGain param, default 0 = off).

Country roads, Cheddar, routes 00000275/276/277 (76 corners with confident lines, R 40-250 m,
30-90 km/h), 2026-11-13:
- The model's request matches the road curvature at the look-ahead point (within 0.1 m/s^2 in gentle
  bends, 0.2-0.3 in tight ones). What is off is the lateral position: the car starts to move to the
  inside more than 2 s before the apex, sits 0.27-0.65 m inside the lane centre at the apex and swings
  back out by 0.3-0.75 m. In left bends that is the centre line, on the exit the road border.
- The model holds its own line like a fast spring: 0.49 m/s^2 per metre of offset up to 1.5 m/s^2 of
  request, 0.32 above 2.5 (open loop on the recorded video with the view shifted 0.3 m), settled in
  0.2 s. So a correction of a few tenths of a m/s^2 moves the car without a fight.

request += gain * (-KP * offset - KD * offset rate), in bends with confident lane lines.

What did NOT work in the closed-loop replay on the logged lines (do not retry without new data):
- Replacing the request by a lane-following one (lane-centre curvature as feed-forward): overshoots to
  the outside by 0.5-1.1 m, unstable at twice the gain.
- A scheduled outward bias without feedback (from the previewed bend strength, also fitted to the
  logged offsets): smooth, but trades the apex for a wider exit; the offset is only 35-60 %
  predictable from the bend shape.
- Heading from the lane lines (the LaneCentreAssist fault, Sep 14): correlation 0.35-0.59 with the
  true heading and biased with curvature. The rate comes from the observer below instead.

Offset and rate come from a two-state observer: predicted with the gyro yaw rate and the lane-centre
curvature the lines showed at the look-ahead point 0.4 s ago, corrected with the near-field offset.
At OBS_W0 = 3 rad/s the added jerk is at the floor a noise-free offset would give.

Replay with this module in the loop, gain 1.0 (R 40-80 / 80-120 / 120-250 m): apex inside
0.46/0.65/0.27 -> 0.18/0.32/0.12 m, largest outside 0.29/0.17/0.24 -> 0.32/0.22/0.10 m, worst body gap
to the centre line in left bends -0.14/-0.14/-0.52 -> +0.11/+0.11/-0.17 m, to the outside line
-0.45/-0.17/-0.46 -> -0.12/-0.15/-0.02 m. Cost: 0.5-0.8 m/s^3 of added request jerk (the request's
own: 0.4-1.0), peak correction 0.35-0.65 m/s^2. Gain 0.5 gives about half of both, 1.5 hits the cap.
The replay is linear around the logged drive with the measured spring as the model: no torque limits.

Not covered: bends below R 40 m, bends without confident lines, torque-limited corners.
The target is the real lane centre: the camera offset modeld applies (static CameraOffset plus the
lane-centering one) is taken out of the measured offset, as in the replay. So in bends the trim works
against a lane-centering offset and roughly halves it (KP against the model's 0.49 m/s^2 per m);
targeting the shifted centre instead left the worst outside-line gap at -0.35/-0.32/-0.23 m.
Signs: y, offset and lateral acceleration are right-positive, like desiredCurvature in this fork.
"""
import numpy as np

DT = 0.05  # s, model rate
KP = 0.5  # m/s^2 per m of offset
KD = 1.0  # m/s^2 per m/s of offset rate
GAIN_MAX = 1.5
MAX_CORRECTION = 1.0  # m/s^2
OBS_W0 = 3.0  # rad/s, observer bandwidth
OBS_RESET = 0.5  # m: a larger jump of the measured offset is a new line, not motion
LOOKAHEAD_S = 0.38  # the curvature at v * this is where the car is ROAD_DELAY frames later
ROAD_DELAY = 8
FIT_HALF = 7.5  # m, half window of the local quadratic at the look-ahead point
BEND_BP = [0.5, 1.0]  # m/s^2 of previewed lateral acceleration: off below, full above
BEND_HOLD_S = 1.5
RAMP_S = 0.7
PROB_ON, PROB_OFF = 0.6, 0.4
SPEED_BP = [8.0, 10.0, 22.0, 25.0]  # m/s: the replay covers 29-90 km/h; above that the split path takes over
SPEED_W = [0.0, 1.0, 1.0, 0.0]
XS = np.arange(0.0, 82.5, 2.5)  # m, grid of the fits


def _curvature(center: np.ndarray, mask: np.ndarray) -> float:
  return float(np.polyfit(XS[mask], center[mask], 2)[0] * 2.0)


class BendTrim:
  def __init__(self):
    self.reset()

  def reset(self) -> None:
    self.offset: float | None = None  # observer: car right of the lane centre (m) ...
    self.rate = 0.0  # ... and its rate (m/s)
    self.road = [0.0] * (ROAD_DELAY + 1)  # lane-centre curvature at the look-ahead point, delay line
    self.bend = 0.0
    self.weight = 0.0  # applied share (0..1), for the log
    self.correction = 0.0  # applied correction (m/s^2), for the log
    self.lines_ok = False

  def update(self, lat_accel: float, x_idxs: np.ndarray, left_y: np.ndarray, right_y: np.ndarray, left_prob: float, right_prob: float,
             yaw_rate_measured: float | None, v_ego: float, gain: float, active: bool = True, camera_offset: float = 0.0) -> float:
    """lat_accel plus the trim. Call once per model frame, also at gain 0, so that the observer is
    settled when the live param switches it on. active = lateral control on and no lane change.
    camera_offset = modelDataV2SP.cameraOffset (m, positive = model centre left)."""
    values = (lat_accel, v_ego, left_prob, right_prob, camera_offset, yaw_rate_measured if yaw_rate_measured is not None else np.nan)
    if not active or not all(np.isfinite(values)) or not (np.all(np.isfinite(left_y)) and np.all(np.isfinite(right_y))):
      self.reset()
      return lat_accel
    v = max(v_ego, 1.0)
    center = 0.5 * (np.interp(XS, x_idxs, left_y) + np.interp(XS, x_idxs, right_y))
    measured = -float(center[0]) - camera_offset

    look = v * LOOKAHEAD_S
    near = np.abs(XS - look) <= FIT_HALF
    k_look = _curvature(center, near if near.sum() >= 4 else XS <= 2 * FIT_HALF)
    k_far = _curvature(center, (XS >= v * 0.5) & (XS <= max(v * 2.5, 30.0)))
    self.road.append(k_look)
    k_road = self.road.pop(0)

    if self.offset is None or abs(measured - self.offset) > OBS_RESET:
      self.offset, self.rate = measured, 0.0
    self.offset += self.rate * DT
    self.rate += (v * float(yaw_rate_measured) - v * v * k_road) * DT
    innovation = measured - self.offset
    self.offset += 2.0 * OBS_W0 * DT * innovation
    self.rate += OBS_W0 * OBS_W0 * DT * innovation

    self.lines_ok = min(left_prob, right_prob) > (PROB_OFF if self.lines_ok else PROB_ON)
    previewed = max(abs(k_look), abs(k_far)) * v * v
    self.bend = max(float(np.interp(previewed, BEND_BP, [0.0, 1.0])), self.bend - DT / BEND_HOLD_S)
    on = gain > 0.0 and np.isfinite(gain)
    target = self.bend * float(np.interp(v_ego, SPEED_BP, SPEED_W)) if (on and self.lines_ok) else 0.0
    self.weight += float(np.clip(target - self.weight, -DT / RAMP_S, DT / RAMP_S))
    if not on:
      self.weight = self.correction = 0.0
      return lat_accel
    self.correction = float(np.clip(self.weight * min(gain, GAIN_MAX) * (-KP * measured - KD * self.rate), -MAX_CORRECTION, MAX_CORRECTION))
    return lat_accel + self.correction
