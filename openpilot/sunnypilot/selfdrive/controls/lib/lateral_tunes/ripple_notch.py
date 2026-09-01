"""
Curvature-ripple notch for the Ioniq 6 model ripple, plus an inert frequency monitor.

The driving model emits a narrowband ripple on desired curvature. Measured on route
000001a4 (engaged, >=15 m/s, 7 runs Welch-averaged) against a fitted power-law background,
the excess over background is:

    0.60 Hz  1.8x      0.69 Hz  14.8x  <-- ripple
    0.65 Hz  3.4x      0.74 Hz   6.5x
                       0.81 Hz   0.9x

and the steering angle shows the SAME 14.8x bump at 0.69 Hz, so this is the frequency the
driver actually feels. It is present open-loop in modelV2 during manual driving, so it is
the model's own output, not a control-loop oscillation.

This module removes that content from the REFERENCE. Two properties make that safe:

  * The measurement is untouched, so no amount of filtering here can cost phase margin.
    A mis-placed notch mis-shapes the command; it cannot destabilise the loop.
  * Real roads carry almost nothing in this band -- 0.5-1.0 Hz is 0.3-3.4% of curvature
    energy at every speed measured -- so the path cost is small.

Why a notch and not the first-order low-pass this replaces: a 0.3 Hz LP has ~0.53 s of
group delay at every frequency below its corner, so it taxed real corner-entry response to
attenuate one narrow band. Measured across the high-speed runs of 000001a4, applying each
filter exactly as it would run on-car:

    filter                        ripple 0.6-0.8 Hz kept    road <0.3 Hz kept
    LP 0.3 Hz, blended 20-28 m/s          90.8%                   99.9%
    notch 0.69 Hz Q=2, from 15 m/s         7.2%                   98.4%

The LP's 90.8% is not a typo: its speed blend only begins at 20 m/s and only reaches full
at 28 m/s, so at the 70 km/h where the weave is actually felt it was doing nothing at all.

DO NOT peak-pick this band without removing the background first. Curvature spectra go as
f^-3, so a plain argmax over 0.4-1.1 Hz returns the lowest bin in the search range rather
than the ripple. That mistake produced an earlier "0.51-0.69 Hz" estimate in which every
number was the background skirt.

## Why the frequency is NOT adapted

A self-tuning version was built and validated offline against the full route before being
rejected on its own numbers. Replaying 000001a4 through the real estimator:

    frequency   depth        ripple kept   road kept
    fixed       full             7.2%        98.4%
    fixed       adaptive        33.2%        99.4%
    adaptive    full            21.6%        98.2%
    adaptive    adaptive        44.4%        99.4%

Both adaptive mechanisms make the filter worse:

  * Frequency. A 90 s window estimates the peak with an inter-quartile spread of 0.128 Hz,
    which is comparable to the notch's own -3 dB width. Tracking that estimate pulls the
    null off the bump more often than onto it.
  * Depth. Keying depth on how strongly the ripple stands out means it fades on straight
    roads -- which is exactly where the weave is most noticeable. The metric and the
    symptom are anti-correlated.

The estimator is therefore retained as a MONITOR only: it publishes what it would have
chosen, and nothing reads it back into control. That still solves the problem adaptivity
was meant to solve -- noticing that a new driving model's ripple has moved -- without
paying for per-window noise. If a future model's logged `measured_hz` sits somewhere else
for a whole drive, re-centre RIPPLE_NOTCH_HZ by hand and re-run this validation.
"""
import numpy as np

# --- notch (acts on control) ---
RIPPLE_NOTCH_HZ = 0.69
RIPPLE_NOTCH_Q = 2.0     # -3 dB width f0/Q = 0.35 Hz; covers the 0.62-0.78 bump with margin
# Speed blend. Starts below the 70 km/h band where the weave is felt; the old LP did not
# begin until 20 m/s, which is why it never touched the symptom being complained about.
RIPPLE_NOTCH_SPEED_BP = [14.0, 16.0]   # m/s
RIPPLE_NOTCH_BLEND_V = [0.0, 1.0]

# --- monitor (logged, never actuated) ---
# Search band must cover every model family we might run, not just the small model's
# 0.69 Hz bump: the chestnut big models put their ripple well below the old 0.45 Hz floor
# (BMV4 measures 0.40-0.47 Hz, an earlier chestnut checkpoint 0.27 Hz), so a monitor
# clamped to 0.45-1.05 was structurally blind to them and reported nothing. The background
# fit band widens with it to keep the bump well inside the fitted region.
RIPPLE_SEARCH_HZ = (0.20, 1.05)
RIPPLE_BACKGROUND_FIT_HZ = (0.10, 2.0)
RIPPLE_CLAMP_HZ = (0.50, 1.00)
ESTIMATOR_TARGET_RATE_HZ = 5.0
ESTIMATOR_WINDOW_S = 90.0
ESTIMATOR_NPERSEG = 384
ESTIMATOR_UPDATE_S = 5.0
ESTIMATOR_MIN_SPEED = 15.0
ESTIMATOR_MIN_IN_BAND = 0.95
ESTIMATOR_MAX_SLEW_HZ_S = 0.01
ESTIMATOR_CONSECUTIVE = 3
# `excess` is the peak of the background-normalised spectrum: how many times above the
# local power-law background the bump stands. Qualifying windows on 000001a4 measure a
# median of ~7.6; flat windows sit at ~1.
ESTIMATOR_MIN_EXCESS = 3.0


class NotchFilter:
  """RBJ biquad notch, direct form I. Unity gain at DC and at Nyquist."""

  def __init__(self, dt: float, f0: float, q: float):
    self.dt = dt
    self.q = q
    self.f0 = 0.0
    self.x1 = self.x2 = self.y1 = self.y2 = 0.0
    self.set_frequency(f0)

  def set_frequency(self, f0: float) -> None:
    if abs(f0 - self.f0) < 1e-6:
      return
    self.f0 = f0
    w0 = 2.0 * np.pi * f0 * self.dt
    alpha = np.sin(w0) / (2.0 * self.q)
    cw = np.cos(w0)
    a0 = 1.0 + alpha
    self.b0 = 1.0 / a0
    self.b1 = -2.0 * cw / a0
    self.b2 = 1.0 / a0
    self.a1 = -2.0 * cw / a0
    self.a2 = (1.0 - alpha) / a0

  def reset(self, x: float) -> None:
    # A notch passes DC at unity, so a constant x is its own steady state.
    self.x1 = self.x2 = self.y1 = self.y2 = x

  def update(self, x: float) -> float:
    y = (self.b0 * x + self.b1 * self.x1 + self.b2 * self.x2
         - self.a1 * self.y1 - self.a2 * self.y2)
    self.x2, self.x1 = self.x1, x
    self.y2, self.y1 = self.y1, y
    return float(y)


class RippleFrequencyMonitor:
  """Rolling-window spectral peak tracker. Observational only -- see the module docstring.

  Deliberately a slow batch estimate rather than an LMS/lattice adaptive notch: the
  quantity is a per-model constant, not something that drifts within a drive, so
  per-sample convergence buys nothing and costs auditability. Every input to every
  decision here is replayable offline against an rlog.
  """

  def __init__(self, dt: float):
    self.decim = max(1, int(round(1.0 / (ESTIMATOR_TARGET_RATE_HZ * dt))))
    self.rate = 1.0 / (dt * self.decim)
    self.n = int(round(ESTIMATOR_WINDOW_S * self.rate))
    self.nperseg = min(ESTIMATOR_NPERSEG, self.n)
    self.update_every = max(1, int(round(ESTIMATOR_UPDATE_S * self.rate)))
    self.max_slew = ESTIMATOR_MAX_SLEW_HZ_S * ESTIMATOR_UPDATE_S

    self.buf = np.zeros(self.n)
    self.in_band = np.zeros(self.n)
    self.idx = 0
    self.filled = 0
    self._acc = 0.0
    self._acc_in_band = 0.0
    self._acc_n = 0
    self._since_update = 0

    self.f_hz = RIPPLE_NOTCH_HZ
    self.measured_hz = 0.0
    self.excess = 0.0
    self.qualifying = 0
    self.updates = 0

    self._window = np.hanning(self.nperseg)
    self._freqs = np.fft.rfftfreq(self.nperseg, 1.0 / self.rate)
    self._search = (self._freqs >= RIPPLE_SEARCH_HZ[0]) & (self._freqs <= RIPPLE_SEARCH_HZ[1])
    self._fit_band = ((self._freqs >= RIPPLE_BACKGROUND_FIT_HZ[0]) &
                      (self._freqs <= RIPPLE_BACKGROUND_FIT_HZ[1]))

  def update(self, value: float, v_ego: float) -> None:
    if not np.isfinite(value):
      return

    # Decimate by averaging, which doubles as the anti-alias filter.
    self._acc += value
    self._acc_in_band += 1.0 if v_ego >= ESTIMATOR_MIN_SPEED else 0.0
    self._acc_n += 1
    if self._acc_n < self.decim:
      return

    # Samples are pushed unconditionally so the buffer's time base stays uniform; the
    # speed gate is applied as a fraction of the window instead. Dropping out-of-band
    # samples would splice together non-adjacent times and smear the peak.
    self.buf[self.idx] = self._acc / self._acc_n
    self.in_band[self.idx] = self._acc_in_band / self._acc_n
    self.idx = (self.idx + 1) % self.n
    self.filled = min(self.filled + 1, self.n)
    self._acc = 0.0
    self._acc_in_band = 0.0
    self._acc_n = 0

    self._since_update += 1
    if self._since_update < self.update_every or self.filled < self.n:
      return
    self._since_update = 0
    self._estimate()

  def _welch(self, x: np.ndarray) -> np.ndarray:
    """Averaged periodogram, 50% overlap. Hand-rolled so the control path takes no scipy
    import; absolute scale is irrelevant because the spectrum is normalised."""
    step = self.nperseg // 2
    acc = np.zeros(self.nperseg // 2 + 1)
    count = 0
    for i in range(0, len(x) - self.nperseg + 1, step):
      seg = x[i:i + self.nperseg]
      acc += np.abs(np.fft.rfft((seg - seg.mean()) * self._window)) ** 2
      count += 1
    return acc / max(count, 1)

  def _estimate(self) -> None:
    # `measured_hz` and `excess` are telemetry and are published UNCONDITIONALLY whenever a
    # spectrum exists. Only `qualifying` (and therefore any notch retune) is gated. The
    # previous version returned before assigning `measured_hz` on every gate failure, so a
    # route whose ripple never cleared the gate logged a flat 0.0 -- indistinguishable from
    # "no ripple" and useless for diagnosing an unfamiliar model.
    if self.in_band.mean() < ESTIMATOR_MIN_IN_BAND:
      self.qualifying = 0
      self.excess = 0.0
      return

    spec = self._welch(np.roll(self.buf, -self.idx))

    # Divide out the f^-3 road background before looking for a bump. Fitting the power law
    # over a band far wider than the bump keeps the bump itself out of the fit.
    coef = np.polyfit(np.log(self._freqs[self._fit_band]),
                      np.log(np.maximum(spec[self._fit_band], 1e-30)), 1)
    background = np.exp(np.polyval(coef, np.log(np.maximum(self._freqs, 1e-9))))
    band = (spec / np.maximum(background, 1e-30))[self._search]
    freqs = self._freqs[self._search]
    self.excess = float(band.max())

    # Power-weighted centroid of the excess, not argmax: same median against the offline
    # truth but 35% less spread window to window (IQR 0.102 vs 0.156 Hz on 000001a4).
    weight = np.maximum(band - 1.0, 0.0) ** 2
    if weight.sum() > 0.0:
      self.measured_hz = float((freqs * weight).sum() / weight.sum())

    if self.excess < ESTIMATOR_MIN_EXCESS or weight.sum() <= 0.0:
      self.qualifying = 0
      return

    self.qualifying += 1
    if self.qualifying < ESTIMATOR_CONSECUTIVE:
      return

    target = float(np.clip(self.measured_hz, *RIPPLE_CLAMP_HZ))
    self.f_hz += float(np.clip(target - self.f_hz, -self.max_slew, self.max_slew))
    self.updates += 1
