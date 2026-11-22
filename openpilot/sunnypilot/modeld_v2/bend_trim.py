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

request += gain * (-KP * offset - KD * offset rate), in bends with confident lane lines (KD = 0 since Nov 22, below).

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

Not covered: bends below R 40 m, torque-limited corners.

KD = 0 since Nov 22 (routes 28c/28e, four bookmarks, all 0.4-0.5 Hz swings in country bends at 62-77 km/h):
the replay above was linear with no steering delay. In the full closed loop (opsim closedloop3 with this
module, an EPS refitted at 67 km/h and the measured vehicle gain) the trim at gain 1.0 reproduces the
bookmarks; without it there is no swing. The rate term pushes hardest at 0.3-0.8 Hz, where the lane loop
has its crossover and ~2-3 dB of gain margin. Cheddar, 246 s of engaged country bends on five other
windows (inside offset p50/p90 | request swing 0.3-0.8 Hz p90 | wheel p90 | request jerk p95):
off 35/89 cm | 0.064 m/s^2 | 0.63 deg | 0.40 m/s^3; KD 1.0: 21/61 | 0.226 | 1.94 | 0.68;
KD 0: 14/62 | 0.077 | 0.76 | 0.42. A 0.15 Hz low-pass on the correction (KD 1) stopped the swing as well
but kept less of the apex (21/63). The observer still estimates the rate (logged, not applied).

Line dropouts (LatBendTrimDropout, default 0 = off): at night on the Allner hill (route 283) both lines
were confident in only 35 % of the bend time, so the trim was on 35 % instead of 50 % (281, daylight).
With dropout on, one confident line (> SINGLE_PROB) gives the centre as that line -/+ half the last
confident lane width, and with no confident line the weight holds for HOLD_S while the observer runs
on gyro and the last road curvature alone. Replay (opsim/lc/sim5.py, Allner hill, trim 1.0): trim on
in bends 35 -> 69 % (283), over the right line 4.6 -> 3.1 %, over the centre line 3.4 -> 2.9 %, worst
centre-line gap -0.72 -> -0.46 m; 281: 1.5 -> 1.2 % / 1.9 -> 1.7 %. Cost: jerk 0.32 -> 0.50 m/s^3 (283).
Scored on the logged lines, which are the weak ones there.
The target is the real lane centre: the camera offset modeld applies (static CameraOffset plus the
lane-centering one) is taken out of the measured offset, as in the replay. So in bends the trim works
against a lane-centering offset and roughly halves it (KP against the model's 0.49 m/s^2 per m);
targeting the shifted centre instead left the worst outside-line gap at -0.35/-0.32/-0.23 m.
Signs: y, offset and lateral acceleration are right-positive, like desiredCurvature in this fork.
"""
import numpy as np

DT = 0.05  # s, model rate
KP = 0.5  # m/s^2 per m of offset
KD = 0.0  # m/s^2 per m/s of offset rate; 1.0 until Nov 22 (drove a 0.4-0.5 Hz swing in bends, see the docstring)
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
HOLD_S = 1.5  # s, dropout: weight held and observer dead-reckoned with no confident line
SINGLE_PROB = 0.6  # dropout: one line above this stands in for both
WIDTH_DEFAULT = 3.0  # m, lane width until both lines were seen
WIDTH_RANGE = (2.2, 4.5)  # m, plausible widths for the memory
WIDTH_ALPHA = 0.05  # per frame, lane width memory


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
    self.lost_s = 0.0
    self.width = WIDTH_DEFAULT
    self.k_last = 0.0

  def update(self, lat_accel: float, x_idxs: np.ndarray, left_y: np.ndarray, right_y: np.ndarray, left_prob: float, right_prob: float,
             yaw_rate_measured: float | None, v_ego: float, gain: float, active: bool = True, camera_offset: float = 0.0,
             dropout: bool = False) -> float:
    """lat_accel plus the trim. Call once per model frame, also at gain 0, so that the observer is
    settled when the live param switches it on. active = lateral control on and no lane change.
    camera_offset = modelDataV2SP.cameraOffset (m, positive = model centre left).
    dropout = keep trimming through line dropouts (single line, then HOLD_S of dead reckoning)."""
    values = (lat_accel, v_ego, left_prob, right_prob, camera_offset, yaw_rate_measured if yaw_rate_measured is not None else np.nan)
    if not active or not all(np.isfinite(values)) or not (np.all(np.isfinite(left_y)) and np.all(np.isfinite(right_y))):
      self.reset()
      return lat_accel
    v = max(v_ego, 1.0)
    ly, ry = np.interp(XS, x_idxs, left_y), np.interp(XS, x_idxs, right_y)
    both = min(left_prob, right_prob) > (PROB_OFF if self.lines_ok else PROB_ON)
    single = dropout and not both and max(left_prob, right_prob) > SINGLE_PROB
    if both and WIDTH_RANGE[0] < ry[0] - ly[0] < WIDTH_RANGE[1]:
      self.width += WIDTH_ALPHA * (float(ry[0] - ly[0]) - self.width)
    if single:
      center = ly + 0.5 * self.width if left_prob >= right_prob else ry - 0.5 * self.width
    else:
      center = 0.5 * (ly + ry)
    measured = -float(center[0]) - camera_offset

    look = v * LOOKAHEAD_S
    near = np.abs(XS - look) <= FIT_HALF
    k_look = _curvature(center, near if near.sum() >= 4 else XS <= 2 * FIT_HALF)
    k_far = _curvature(center, (XS >= v * 0.5) & (XS <= max(v * 2.5, 30.0)))
    seen = both or single
    self.lost_s = 0.0 if seen else self.lost_s + DT
    holding = dropout and not seen and self.lines_ok and self.lost_s <= HOLD_S  # only right after lines (or a hold)
    if seen:
      self.k_last = k_look
    elif holding:  # the lines' shape is not trusted: keep the last road curvature
      k_look = k_far = self.k_last
    self.road.append(k_look)
    k_road = self.road.pop(0)

    if not holding and (self.offset is None or abs(measured - self.offset) > OBS_RESET):
      self.offset, self.rate = measured, 0.0
    self.offset += self.rate * DT
    self.rate += (v * float(yaw_rate_measured) - v * v * k_road) * DT
    if holding:
      measured = self.offset
    else:
      innovation = measured - self.offset
      self.offset += 2.0 * OBS_W0 * DT * innovation
      self.rate += OBS_W0 * OBS_W0 * DT * innovation

    self.lines_ok = seen or holding
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
