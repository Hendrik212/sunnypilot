# Ioniq 6 StarPilot lateral tuning history

Living log of what we changed, what the car did, and which conclusions actually
survived a later route. Append a dated section after every drive that moves a
knob. Do not rewrite old sections to match a new theory — add a correction.

Car: Hyundai Ioniq 6 2023 (firmware `240206` only; the StarPilot 2025 path is
not ported). Dongle `568c82e1de7c61a2`. Device `ioniq_local` (`192.168.1.197`).
Fork: `Hendrik212/sunnypilot` branch `isla-master`, opendbc `sp-isla-master`.

Gate every log analysis on `carControl.latActive` (MADS). `selfdriveState.active`
is ~6% of the same drive and will lie.

## Current tree (2026-08-27)

Last driven: **409 flat + StarPilot PID** (`9f2d11fd` / opendbc `5923cfef`),
route `000001a4` on `6024dfa88`.

Pushed but **not yet driven**: **speed-scheduled 600 envelope with proportional
`latAccelFactor`** (`453e984836` / opendbc `c3ed8a3d`). Peak panda
`max_torque = 600`. Unsaturated mapping held at `409/3.66 ≈ 112` CAN per m/s².

```
v (m/s)     km/h      STEER_MAX   latAccelFactor   friction   CAN / m/s²   friction CAN
≤ 5.0        ≤ 18          409            3.66      0.090            112             37
6.5–15.0    23–54          600            5.37      0.0614           112             37
≥ 17.0       ≥ 61          409            3.66      0.090            112             37
```

PID stays `KP=0.6 / KI=0.35`. Rate stays 10/8 below 17 m/s, ramping to 2/3 by
19.4 m/s (70 km/h). Live torque params remain off. Stock CANFD (tune off) stays
270/2/3.

A panda rebuild/flash is required before the 600 drive. If firmware stays at
409, commands above 409 are silently dropped.

### 2026-08-27 — Fable review of the undriven 600 build: three fixes before the drive

An independent control-system review of the whole tune, cross-checked against a
fresh recomputation from the `000001a4` cache (147906 engaged hands-off frames).
Nothing here has been driven. All three changes land on top of `453e984836`.

**1. The 600 invariance was incomplete: friction's CAN scaled with `STEER_MAX`.**
`get_friction` returns `±friction · latAccelFactor` in lat-accel space
(`opendbc/car/lateral.py`), and the feedforward is divided by `latAccelFactor` on
the way out — the two cancel, so friction's *normalized* torque is exactly
`friction` and its CAN value is `friction · STEER_MAX`. Scaling `latAccelFactor`
held P/I/FF at 112 CAN per m/s² and missed this one term entirely: the breakaway
kick would have gone `0.09·409 = 37` → `0.09·600 = 54` CAN, +46%, on the one
term that is a square wave through every error sign change — in the band that
already flips 1.6–1.8 times a second. `friction_for_speed()` now rides the same
schedule. Note this is **not** a pure restoration: holding friction's CAN
constant shrinks its lat-accel-space contribution inside the 600 band. That is
the right invariant only because 409/3.66 is what was actually tuned and driven.

Lesson #1 was stated as "scale `latAccelFactor` with `STEER_MAX`". That is
necessary but not sufficient — see the amended lesson below.

**2. The 600 window opened at 4.0 m/s, inside the P-relay band.** Per-band
decomposition of `000001a4`:

```
band (m/s)     n      railed%   |p|    |i|    |f|   |dLA|   mean|CAN|
2.8–4.0      1195       58.1   7.33   0.02   0.67   0.47        196
4.0–5.0      2193       32.0   3.85   0.02   0.53   0.29        148
5.0–8.0      5408       25.5   2.52   0.04   0.65   0.56        127
8.0–11.0     5274       36.9   1.79   0.05   1.36   1.48        184
11.0–15.0   25762        3.8   0.36   0.09   0.52   0.63         67
```

Below 5 m/s, P is 6–11× the feedforward — that is lesson #4's relay, and a taller
rail there is a bigger sawtooth. Genuine cornering demand only takes over by
8–11 m/s. Ramp-in moved `2.8→4.0` ⇒ `5.0→6.5` m/s.

**3. `low_speed_reset_threshold` was a degenerate expression.** It read
`min(max(CP.minSteerSpeed, 0.3), 0.0447)`, and `max(x, 0.3) ≥ 0.3` makes the
outer `min` return the constant `0.0447` for every input — both other terms dead.
Now `max()` of all three ⇒ 0.3 m/s. Measured effect on `000001a4`: 6.5 s of
engaged time between 0.0447 and 0.3 m/s where the integrator now resets instead
of running, max `|i|` there 0.181 against a 3.66 clip. Correctness fix, not a
tuning change; the creep relay (0.3–2 m/s) sits above the threshold either way.

#### Measurements that supersede earlier prose

- **Ceiling plateaus are short.** 87 episodes at `|CAN| ≥ 400`, p50 **0.07 s**,
  p90 0.71 s, max 1.43 s. Only 12 episodes ≥ 0.5 s, totalling **11.7 s** — and
  *all* of them sit between **5.7 and 9.6 m/s**. Above 11 m/s the whole route
  contains 1.7 s of transient touches. "16% of tight-corner frames ≥ 400" is
  true but is mostly flip transients, not sustained starvation. On the sustained
  plateaus the deficit is real: desired 2.63 vs actual 2.17 m/s² (a/d 0.82).
- **The plant is nonlinear at the rail.** Those plateaus delivered ~2.17 m/s² at
  ≥400 CAN ≈ 190 CAN per m/s², vs the 112 nominal mapping. The "linear CAN
  p50 = 509" extrapolation assumed that slope persists; if it steepens further,
  600 buys less than the arithmetic suggests. Not settleable until driven.
- **"lag 0 ms" at 70 km/h was broadband and wrong at the ripple frequency.**
  Cross-spectra on three ≥20 s runs in 17.5–21.5 m/s: model→desired gain 1.00,
  phase −6° to −7° (the tune adds nothing to the reference — the
  `expected + jerk·τ` construction is algebraically the identity). But
  desired→actual at the ripple peak is gain 0.90–1.39 at phase **−75° to −108°**
  (0.36–0.45 s, the EPS lag), while broadband cross-correlation reads 0 frames.
  Controller output ripple runs **255–386** against a static `v²/laf` of 104,
  i.e. ~2.5–3.7× static gain, spent fighting a phase-lagged error it cannot win.
  `|f| = 0.414` is not the anomaly — it is the reference itself (`|f|/|dLA| = 0.90`).
- **The ripple frequency does not scale with √v.** Measured peaks: 70 km/h band
  0.575 / 0.661 / 0.670 / 0.687 Hz; 120 km/h band 0.536 / 0.686 Hz. The
  `f ≈ 0.126·√v` law predicts 0.56 → 0.71 Hz across that range and the measured
  peak simply does not move. This kills speed-scheduling the notch and makes a
  **fixed ~0.65 Hz notch** the right shape — it would serve both bands with far
  less phase cost than widening the 0.3 Hz low-pass, which currently eats ~30° at
  real-curve frequencies. Not implemented; wants its own A/B.
- **Corner cutting is the model's path — confirmed harder.** Restricting to
  well-tracked (a/d ∈ [0.95, 1.05]) *unrailed* corner frames, n = 2126: left
  turns **+0.349 m**, right turns **−0.371 m**. Symmetric, on the path, path
  inside. Lesson #5 stands and is now backed by tracking-matched frames rather
  than episode anecdotes.
- **The inside-offset metric cannot judge the 600 change.** An earlier claim here
  that better tracking would deepen the measured cut does not hold: binned by
  a/d, higher a/d goes with *less* inside offset (+0.484 m at a/d 0.5–0.7 →
  +0.089 m at a/d 1.05–1.3, at 1 s lag). But that relation is confounded —
  `dLA` already contains the model's path correction, so a/d > 1 co-occurs with
  recovering *from* being inside. Use plateau episodes to judge 600, not offset.
- **Creep railing is unwind-to-center, not cornering.** 93.5% of railed frames at
  0.3–1.0 m/s have `|dLA| < 0.05` (desired is straight) with mean `|steering
  angle|` 104°. Also: the worst band is **1–2 m/s (49.9% railed, |p| = 12.33)**,
  not 0.3–1.0 m/s, which is only 405 frames. The `KP` table is ≈ `250/v²`, i.e.
  a deliberate constant-gain-in-curvature-space schedule; the LSF multiplier
  `(1 + LSF/KP)` is only 1.05–1.57 and is *not* the dominant low-speed gain.

#### Still open (not implemented)

- Fixed ~0.65 Hz reference notch, replacing the 0.3 Hz low-pass. A/B metric:
  0.6–1.0 Hz band amplitude of `steeringAngleDeg` at 70 km/h.
- Low-speed rework: give the angle-space assist ownership below ~2 m/s instead
  of adding it on top of a railed lat-accel-space PID. Only an angle-space loop
  degrades gracefully to `steerAtStandstill` (everything lat-accel-space
  vanishes as v→0).
- Narrow the 15–17 m/s ramp-out. The data shows 1.7 s of ≥400 CAN above 11 m/s
  in the whole route, so the upper half of the window carries the 600 gain over
  ~26k frames that never ask for it. Deferred until the 600 drive is evaluated.
- `vEgo`/`vEgoRaw` skew: the profile writes `latAccelFactor` from Kalman
  `CS.vEgo`, `carcontroller` rebuilds `STEER_MAX` from `CS.out.vEgoRaw`. Inside
  the ramps the slope is ~160 CAN/(m/s), so a 0.2 m/s disagreement is a
  transient ~5% gain error. The 112-invariance is exact only outside the ramps.
- `PIDController` never re-clips a stored `i` when the limit *shrinks* on band
  exit (`pid.py` bounds growth only). Measured `|i| ≤ 0.24` everywhere, so this
  is theoretical today.
- Route `000001a4` ran the **inverted** `get_lat_delay`, i.e. the stale
  `LagdValueCache` + 0.1, and that cached value was never logged. Every
  jerk-keyed number above is conditioned on an unknown effective τ. Pull the
  applied `lat_delay` from the next route before treating them as comparable.

### 2026-08-27 (later) — the ripple is at 0.69 Hz; LP replaced by a notch; adaptation built, validated, rejected

**The 0.3 Hz low-pass was never active where the weave is felt.** Its blend ran
`[20, 28] m/s`, so at 19.4 m/s (70 km/h) it was at exactly 0.0. Measured across
the high-speed runs of `000001a4`, applying each filter as it would actually run
on-car, the LP removed **9%** of 0.6–0.8 Hz reference energy. It was not
under-performing; it was absent.

**The ripple is at 0.69 Hz, not 0.75 and not 0.51–0.69.** Both earlier numbers
were wrong, in opposite directions and for the same reason: curvature spectra go
as `f^-3`, so `argmax` over a 0.4–1.1 Hz search band returns the *lowest bin*,
not the bump. Every "peak" in the 0.508–0.687 list from earlier today was the
background skirt. Fitting and dividing out the power-law background first gives
an unambiguous answer on `000001a4` (engaged, ≥15 m/s, 7 runs Welch-averaged):

```
excess over fitted background:  0.60 Hz 1.8x   0.65 Hz 3.4x   0.69 Hz 14.8x
                                0.74 Hz 6.5x   0.81 Hz 0.9x
```

The steering angle shows the **same 14.8× bump at 0.69 Hz**, so this is the
frequency the driver feels, not just a number in the command.

**Replaced with a fixed notch, 0.69 Hz, Q=2, blended in over 14–16 m/s.**
Offline, on real logged curvature, causally filtered:

```
filter                              ripple 0.6-0.8 Hz kept   road <0.3 Hz kept
LP 0.3 Hz, blend 20-28 m/s                   90.8%                 99.9%
notch 0.69 Hz Q=2, blend 14-16 m/s            7.2%                 98.4%
```

Centre sweep is sharply peaked at 0.69 (0.62 → 20.8%, 0.65 → 12.6%, **0.69 →
7.2%**, 0.72 → 8.0%, 0.75 → 12.4%), which independently confirms the measurement.
Q sweep: Q=1 → 2.0% ripple / 96.3% road, Q=2 → 7.2% / 98.4%, Q=4 → 21.8% / 99.3%.

#### The adaptive version was built, validated offline, and rejected on its numbers

The plan was a self-learning notch converging from the 0.69 default. It was
implemented and replayed through the full route before any on-car exposure. It
loses to the fixed notch on every axis that matters:

```
frequency   depth        ripple kept   road kept
fixed       full             7.2%        98.4%
fixed       adaptive        33.2%        99.4%
adaptive    full            21.6%        98.2%
adaptive    adaptive        44.4%        99.4%
```

- **Frequency.** A 90 s window estimates the peak with IQR 0.128 Hz, comparable
  to the notch's own −3 dB width (0.35 Hz). Tracking that estimate pulls the null
  off the bump more often than onto it. Slowing the slew to 0.001 Hz/s helps only
  marginally (37.0% kept) and never approaches the fixed 7.2%.
- **Depth.** Keying depth on how strongly the ripple stands out fades the notch on
  straight roads — which is exactly where the weave is most noticeable. The
  metric and the symptom are anti-correlated.

The estimator is kept as an **inert monitor**: it publishes `rippleMeasuredHz` and
`rippleExcess` on `lateralTuneStateSP` and nothing reads them back into control.
That still solves what adaptivity was for — noticing that a new model's ripple
has moved — without paying per-window noise. Gating validated: `f_hz` moved on 2
of 62502 frames below 15 m/s.

Things worth keeping from the exercise, because they are cheap to get wrong again:

1. **Never peak-pick a curvature spectrum without removing the background.**
2. **A power-weighted centroid of the excess beats `argmax`** — same median
   against truth, 35% less window-to-window spread (IQR 0.102 vs 0.156 Hz).
3. **Validate an estimator offline against the corpus before wiring it to
   control.** This one looked completely reasonable and was worse than a constant.

#### Correction to this morning's "the ripple does not scale with sqrt(v)"

That entry above is **wrong**, and wrong for the reason this entry is about: its
peaks (0.575-0.687 at 70 km/h, 0.536-0.686 at 120 km/h) came from `argmax` on a
sloped background. Divided by the fitted background, the peak moves cleanly with
speed -- and *downward*, which is the opposite of the `0.126*sqrt(v)` law that
the original scheduling attempt assumed:

```
band (m/s)   km/h      runs  secs   peak Hz  excess   0.69 notch |H| at peak
15.0-17.5    54-63        1    22    0.741     7.3          0.27
17.5-22.0    63-79        3   134    0.695    14.9          0.03
22.0-28.0    79-101       8   358    0.602     4.9          0.48
28.0-40.0   101-144       3   173    0.463     1.7          0.85
```

Steering angle reproduces the trend independently (0.787 / 0.695 / 0.602 /
0.463), so it is in the plant response, not only in the command.

So "one notch for all speeds" is now measured rather than assumed, and the
measurement is a qualified yes: 0.69 Hz is near-optimal in the 63-79 km/h band
where the ripple is by far the strongest (excess 14.9) and where the weave is
actually reported, and it goes nearly transparent above 101 km/h where the excess
has fallen to 1.7 and there is little left to remove. The weak spot is 79-101
km/h (|H| = 0.48 against excess 4.9).

A speed-scheduled centre is NOT ruled out -- it is unmeasured. The two bands that
carry real evidence are 17.5-22 (134 s) and 22-28 (358 s); the endpoints are a
single 22 s run and an excess of 1.7 that may be background residual. Deliberately
not shipped with this batch: one route, and it would add a fifth stacked change.
Confirm on a second route first.

#### Reading the next route: what is and is not separable

The notch (>=14 m/s) and the 600 envelope (5-15 m/s) are *nearly* disjoint, not
disjoint:

- **70 km/h weave (>=17.5 m/s)** -- notch only. 600 has exited by 17 m/s.
- **22-50 km/h corners (6-14 m/s)** -- 600 only. The notch blend is 0 below 14.
- **50-60 km/h (14-17 m/s)** -- BOTH partially active. Any change in sweepers at
  that speed is confounded between them. Not a primary complaint band, but do not
  read it as evidence for either change.

One more framing trap: the "LP removed 9%" figure is **deployment-weighted**. At
full blend a 0.3 Hz first-order LP removes ~60% at 0.7 Hz; it removed 9% because
its blend was still 0.0 at the speed where the symptom lives. The lesson is that
the blend was in the wrong place, not that low-passes do not work.

#### Still open

- The notch is calibrated to the model shipped 2026-08. On a model update, read
  `rippleMeasuredHz` over a drive; if it has moved, re-run the offline validation
  and move `RIPPLE_NOTCH_HZ` by hand. There is a test pinning the constant to
  0.66–0.72 specifically so this cannot drift unnoticed.
- The notch acts ≥14 m/s. The ripple is present below that too, but the relay
  behaviour under ~8 m/s dominates there and is a separate problem.
- Not yet driven. Stacked with the 600 envelope, the friction schedule, the
  `lat_delay` fix and turn_intent gating. The notch and the 600 envelope act in
  nearly disjoint speed bands (600: 5–15 m/s, notch: ≥14 m/s), so one drive can
  evaluate both by splitting on speed.

## Lessons that keep being expensive

These are the ones we have now paid for more than once.

1. **`STEER_MAX` without scaling `latAccelFactor` is a gain change, not
   headroom.** `carcontroller` does `CAN = torque * STEER_MAX` and
   `torque ≈ lataccel / latAccelFactor`. Raising only the ceiling raises CAN
   per m/s² on every unsaturated command. That was the 500 experiment.
   **Amended 2026-08-27: scaling `latAccelFactor` is necessary but not
   sufficient.** It covers P/I/FF, whose torque is `lataccel/latAccelFactor`. It
   does *not* cover any term whose normalized torque is dimensionless — friction
   is one (`latAccelFactor` cancels), so its CAN scales with the ceiling
   untouched. When changing `STEER_MAX`, enumerate the terms by whether
   `latAccelFactor` survives the divide, and hold each one's CAN separately.
2. **Panda `max_torque` must equal the peak car-layer request.** A mismatch
   clips with no alert. Tests in `test_lateral_tunes.py` read
   `hyundai_canfd.h` and walk 0–40 m/s.
3. **`max_rt_delta` must cover `max_rate * 25` frames in 250 ms.** Raising
   rate to 10 without moving `max_rt_delta` off 112 was the 720/500 wobble
   (`a6095657`, `95dc60fe`): the RT check rejected the message, reset
   `desired_torque_last` to 0, and the wheel dropout-ramped. Current
   `max_rt_delta = 375`.
4. **Creep-band railing is P bang-bang, not missing torque.** A taller rail
   there is a bigger sawtooth. Measured 2026-08-27: it is the `KP` table
   (≈`250/v²`, KP=250 at 1 m/s), *not* the low-speed factor — the LSF multiplier
   `(1 + LSF/KP)` is only 1.05–1.57 across the whole speed range. The relay
   extends to ~8 m/s, not ~3, so 600 now ramps in at 5.0–6.5 m/s.
5. **Desired-path cut toward oncoming is not a torque-authority bug.**
   Left-handers on `000001a4` sat 35 cm left of center with `a/d ≈ 1` and 0%
   rail. More peak torque can make that worse.
6. **Gating a correction on "is the symptom strong right now" can be exactly
   backwards.** Notch depth keyed on ripple prominence fades on straights --
   which is where the weave is most felt. Before gating any correction on a
   measure of its own symptom, check the sign of the correlation between the
   metric and the complaint. Measured cost of getting this wrong: 7.2% -> 33.2%
   of ripple energy kept. Any "only filter when the ripple is strong" scheme has
   this failure mode.
7. **A spectrum with a sloped background cannot be peak-picked directly.**
   Curvature goes as `f^-3`; `argmax` over a search band returns its lowest bin.
   Fit and divide out the background first. This produced two wrong ripple
   frequencies (0.75, then 0.51-0.69) before the right one (0.69).
8. **Offroad / host-boot / constants readout never execute `if active:`.**
   The 2026-08-24 `calibrated_pose` NameError crashed 0.1 s after engage
   with a clean offroad check. Fetch the route before forming a theory
   (`CLAUDE.md` post-mortem).

## Timeline

### 2025 — 720 then 500, stock v2 (pre-StarPilot)

Aggressive CANFD limits (`STEER_MAX` 720, rate 10/10, `max_rt_delta` 90) produced
highway wobble at ~80 km/h and LKAS dash errors. Root cause was the RT-check
reset above, not EPS refusal — this car has run 500 for months and 720 briefly.

Rolled back to 500/6/8 with `max_rt_delta` 112 (`95dc60fe`). Older write-ups
live outside this checkout (`openpilot/TORQUE_FIX_SUMMARY.md`,
`openpilot/TORQUE_DYNAMIC_LIMITS.md` on the Linux box). Treat those as
prehistory; the StarPilot port replaced that control law.

### 2026-08-18 — curvature-ripple prefilter

Ioniq 6 model curvature has a ~0.78 Hz ripple on highway. A reference-side
low-pass (0.3 Hz cutoff) blends in over 20–28 m/s (72–101 km/h). Measurement
is untouched, so it cannot move phase margin. Isolated into the v2 tune in
`39622fa930`. Inert below 72 km/h — it does not explain 70 km/h weave or
low-speed corners.

### 2026-08-21 — speed-scheduled 409 on stock v2

Routes `00000191` / `00000193` (stock CANFD 270/2/3, engaged frames):

| band | ceiling hits | rate-cap lag | I frozen | tracking err |
|---|---|---|---|---|
| 0–8 m/s | 28–50% | 222–357 ms | 55–69% | 0.077–0.159 |
| 8–15 m/s | 11–12% | 109–115 ms | 41–48% | 0.078–0.106 |
| 15–22 m/s | 2.4% | 27 ms | 19–29% | 0.035–0.040 |
| 22–40 m/s | ~0 | 5–7 ms | 6–13% | 0.015–0.023 |

`0be6b7fe` scheduled STEER_MAX 409 below 15 m/s, rate 10/8, panda envelope 409
with `max_rt_delta` 375. Highway stayed 270. This was still a gain change
below 15 m/s (same normalized command → 1.51× CAN) and it applied to every
HKG CANFD platform, not just the Ioniq 6.

### 2026-08-22 — rate schedule to 70 km/h

Ceiling stops mattering above ~15 m/s; the rate cap does not. Routes
`00000191` / `00000193`: 14–33% of 54–70 km/h frames still rate-capped, I
frozen up to 47%. `a72dba09` holds 10/8 to 17 m/s and returns to 2/3 by
19.4 m/s. No panda change (firmware rate already 10). A/B on 193 vs 194:
rate-limit saturation at 54–70 km/h fell 44% → 3% of curve frames with no
tracking cost.

### 2026-08-23 — StarPilot port, switchable

`963c20f101` / opendbc `4edd7cd8`. 2023-path shaping extracted from
StarPilot `latcontrol_vehicle_tunes.py` into `lateral_tunes/ioniq6_shaping.py`,
equivalence-tested bit-identical on ~21k grid points. Selected by
`TorqueControlTune = 2` with `EnforceTorqueControl` on.

First wiring was inside `latcontrol_torque_v2.py` (`_update_ioniq6`). Flat
STEER_MAX 409, rate 10/8 → 2/3 over 17–19.4 m/s, driver allowance 75
(StarPilot ships 100; softened on purpose). Baseline `latAccelFactor = 3.0`,
`IONIQ_6_BASE_LAT_ACCEL_FACTOR_MULT = 1.22` → effective 3.66, i.e. 112 CAN
per m/s² vs a measured plant-neutral ~105.

LateralJerkTorqueController must stay off: that extension recomputes torque
with a hard-coded friction threshold and drops the Ioniq 6 friction scale.

### 2026-08-24 — crash, then port fidelity

Engage crashed controlsd (`NameError: calibrated_pose`) 0.1 s after
`latActive`. Cause: `update()` was split into `_update_upstream` /
`_update_ioniq6` and the extension call moved without the local. Fixed in
`cfe5522928`. Three wrong diagnoses (model JSON bump, radar cherry-pick,
revert the upstream merge) came first. The route traceback was unambiguous.

Same day, the inner loop moved behind a profile (`13fcea4cc0`) so the
upstream path has zero profile indirection. Then the remaining StarPilot
invariants the first port had dropped:

| commit | what | why it mattered |
|---|---|---|
| `cf140f9f` | `use_live_torque_params = False` | torqued was publishing the override.toml seed (2.5 / 0.005, `valid=False`) every frame and replacing the tune. On-device after an hour: `calPerc=55`, friction 0.005 → FF ~5% of intended. |
| `5018ae1e` | LSF floor = `MIN_SPEED` (1.0) | a 0.3 m/s floor makes LSF `(12/0.3)² = 1600` vs StarPilot 144 in the 0.3–1.0 m/s band where lateral is live. |
| `167e6e75` | `lat_delay += 0.1` | StarPilot modeld `LAT_SMOOTH_SECONDS = 0.1`; ours is 0.0. Jerk is `Δlataccel / lat_delay`, so every jerk-keyed constant sat ~22% hot (measured `lateralDelay` 0.4585 s). |
| `f09ab9d8` | inactive branch resets PID and tracks `steeringPressed` | re-engage from a wound integrator / unprimed buffer is a hard unwind shove at creep gains. |

Tune selection is derived from params every rebuild, never a stored CP_SP
flag, so card (10 Hz) and controlsd (~1 Hz) cannot disagree for longer than
their poll intervals.

### 2026-08-25 — 500 envelope, then the route that killed it

`b707cbb2` / `b6ffcf6f`. Panda 500 flat; car layer 500 → 409 across 15–17 m/s.
The commit message already said this was **not free headroom**: 500/3.66 = 137
CAN per m/s² vs 409/3.66 = 112, i.e. every low-speed command +22%. That 1.22
exactly undoes `IONIQ_6_BASE_LAT_ACCEL_FACTOR_MULT`.

Justification at the time: route `0000019b` full-ceiling frames were 4.3%
below 3 m/s and 0% above 15 m/s, and the profile now owns a fixed 3.66 so
torqued cannot desynchronise. EPS was known not to be the constraint.

### 2026-08-26 — route `000001a1` (the 500 drive)

Public route `568c82e1de7c61a2/000001a1--ac71575222`. Git `d9fd7031`,
`profileId=ioniq6_starpilot`, applied `laf=3.66 / friction=0.09` every frame.
Torqued still on the seed (`valid=0`, `calPerc=29`) and correctly ignored.
PID on this build was still v2's `KP=1.0 / KI=0.3` (highway-only divergence;
below 15 m/s the interp table matches StarPilot).

Gate `carControl.latActive`. `CAN == actuatorsOutput.torque * STEER_MAX(v)`
exactly — panda was not secretly clipping. The gap is between controller
output and `actuatorsOutput`: the 10/8 rate limiter.

**Early, hands-off, no blinker**

| band | \|out\|>0.97 | CAN≥490 | 0.1 s steer HP | output flips/s | mean \|dLA\| when railed |
|---|---|---|---|---|---|
| 1–3 m/s | 54% | 17% | 3.45° | 2.0 | 0.34 |
| 3–8 m/s | 35% | 5% | 3.21° | 1.4 | 0.67 |
| 8–15 m/s | 15% | 5% | 1.41° | 1.3 | 2.31 |

At 1–8 m/s, 31% of engaged frames are controller at ±1.0 with CAN at ~half
envelope (mean 251). Railed 3–8 m/s: `|p|=7.27`, `|f|=0.93`, `|i|≈0` — 100%
P-dominated. `p+i+f = 8.15` against a 3.66 clip, logged error 0.60, desired
only 0.67. `|cs.out|−|actuatorsOutput.torque|` mean 0.44, p90 0.90. Bang-bang
P, rate-limited into a ~1.4 Hz sawtooth.

Desired lataccel never exceeded 3.48 (the 3.66 “accel ceiling” story is
false). CAN-clipped-but-controller-not-railing is ~0.3% — the ceiling was
almost never the bind. 500 made the sawtooth bigger (50 frames 0→500 vs 41
to 409) against the same 10/8 rate.

Corners `|dLA|>1.5`: 27 episodes, 9 undershoot / 1 overshoot, mean peak
actual −0.19 vs desired.

Highway: no saturation. 15–20 m/s (ripple filter off) 0.22° HP; 20–28 blend
0.047°; 28+ 0.034°. Peak ~0.2–0.4 Hz, not the 0.78 Hz the prefilter targets.

**What shipped the same day:** revert STEER_MAX to 409 flat, set profile PID
to StarPilot `KP=0.6 / KI=0.35` (`9f2d11fd` / `5923cfef`). Host boot-to-UI
gate added (`6024dfa88`) so a cereal/import crash dies here before a device
pull.

### 2026-08-26 — route `000001a4` (409 + StarPilot PID)

31 segments, git `6024dfa88`, `laf=3.66 / friction=0.09`, `|CAN| max=409`.
Driver report beforehand: highway okay-ish, 70 km/h ping-pong, low/mid
corners bad, suspected cut toward oncoming.

**Does the car follow the desired path?** Yes, when it has torque. Engaged
hands-off tracking `corr` 0.89–0.99 across all speed bands.

| band km/h | n | corr | a/d | rail% | \|CAN\| max | lane offset m |
|---|---|---|---|---|---|---|
| 4–11 | 369 | 0.97 | 0.61 | 24.4 | 409 | +0.21 |
| 11–29 | 7190 | 0.89 | 0.95 | 30.8 | 409 | +0.03 |
| 29–54 | 30044 | 0.97 | 0.91 | 8.8 | 409 | +0.05 |
| 54–63 | 14134 | 0.99 | 0.97 | 0.4 | 365 | −0.02 |
| 63–76 | 15279 | 0.99 | 0.98 | 0.0 | 286 | −0.04 |
| 76–86 | 16848 | 0.97 | 0.94 | 0.0 | 280 | −0.02 |
| 86–101 | 35409 | 0.94 | 0.89 | 0.0 | 125 | −0.02 |
| 101–144 | 18460 | 0.99 | 1.00 | 0.0 | 126 | −0.12 |

Lane offset is `(leftY + rightY) / 2` with y-right-positive. Positive = car
left of center = toward oncoming on DE roads.

**Tight corners** (`|dLA|>1.2`, 3–16 m/s, 8336 frames, mean 41 km/h):

- 39% railed. `max|CAN|=409`.
- mean `|dLA|=2.13`, `|aLA|=1.83`, `|a|/|d|` p50=0.91.
- 35 episodes: 19 undershoot / 2 overshoot. Mean peak actual −0.22 vs desired.
- Frames actually sitting on the CAN rail (`|CAN|≥400`): desired **2.91** vs
  actual **2.35**, `a/d=0.80`. Linear CAN to match: **p50=509 / p90=625**.
- All railed hard-corner frames (including those still climbing): mean
  `|CAN|=323`. Rate 10/8 is a real second bind — extra ceiling does nothing
  until the slew arrives (~0.6 s 0→600 at 100 Hz).

**0–10 km/h:** rail 10% overall, 28% in the moving 3–10 km/h slice, at
`|dLA|` 0.03–0.13. Same P/LSF bang-bang as `000001a1`. Not a torque deficit.

**70 km/h (17.5–21.5 m/s):** `n=18170`, mean 70.2 km/h, **0% rail**, tracking
corr 0.99, lag 0 ms. Curvature HP: model = controller des = plant (ratio
1.05). The weave is already in the desired path. Ripple prefilter is still
off until 20 m/s (72 km/h).

**Cutting toward oncoming, 3–16 m/s, `|dLA|>0.8`:**

- Left-handers (`dLA<0`): mean offset **+0.35 m** (car left of center).
- Right-handers (`dLA>0`): mean offset **−0.25 m** (car right of center).
- Symmetric inside bias, not a one-sided “missing left torque” story.

Worst left-hander for replay: seg 16 +11 s, ~73 km/h, `a/d=1.01`, rail 0%,
offset +0.90 m / p90 +1.08 m — the car is on the path, the path is in the
oncoming lane. Replay: `#1 16 +11s -s 968`; `#2 24 +32s -s 1469`;
`#5 22 +58s -s 1375 a/d=0.49 rail 94%` (the last one is the torque-limited
corner, not the cut).

**Decision:** raise peak torque for the 22–54 km/h corners that sat on 409,
without repeating 500. Schedule 600 only in 4–15 m/s, keep 409 at creep and
from 17 m/s up, and scale `latAccelFactor` with STEER_MAX so unsaturated
CAN/m/s² stays 112. 600 covers the linear-CAN median and ~96% of p90 if the
plant is linear past 409; 625 was never a commanded value.

### 2026-08-27 — 600 proportional envelope (not yet driven)

Implemented in:

- `opendbc/sunnypilot/car/hyundai/lateral_limits.py` — schedule +
  `lat_accel_factor_for_speed`
- `opendbc/safety/modes/hyundai_canfd.h` — `max_torque = 600`
- `opendbc/safety/tests/test_hyundai_canfd.py` — `MAX_TORQUE_LOOKUP`
- `lateral_tunes/ioniq6_starpilot.py` — `_apply_speed_scheduled_factor` on
  init / inactive / update, calls `update_limits()`
- `lateral_tunes/tests/test_lateral_tunes.py` — schedule, panda peak, gain
  ratio `STEER_MAX / laf == 409/3.66`

550 was considered first; p90=625 made 600 the next ceiling.

Committed and pushed as `453e984836` (opendbc `c3ed8a3d`) on 2026-08-27. Not
driven yet — the panda flash is still required first.

## What each knob actually does

`CAN = (lataccel / latAccelFactor) * STEER_MAX`, clipped at ±1.0 torque
(`steer_max` in the PID is normalized). PID limits expand with
`latAccelFactor` via `update_limits()`.

| change | unsaturated commands | railed commands | known failure |
|---|---|---|---|
| Raise STEER_MAX, leave laf | hotter (gain ↑) | more CAN | 500 at 3.66 |
| Raise both, keep STEER_MAX/laf | unchanged | more CAN | the 600 experiment |
| Raise laf only | weaker torque | weaker rail | do not |
| Raise rate 10/8, leave `max_rt_delta` 112 | — | slew then dropout | 720 wobble |
| Creep STEER_MAX > 409 | bigger sawtooth | not more path | `000001a1` 1–8 m/s |

PID `KP`/`KI` below 15 m/s is already StarPilot’s interp table. The 0.6 vs
1.0 divergence is highway-only (+67% at 30 m/s). `000001a4` did not need
that fight; leave it at 0.6.

## Next drive — what to look at

After pulling 600 (panda flashed):

1. Tight corners 14–54 km/h: `|a|/|d|` on `|CAN|≥550` frames, and whether
   rail% at `|dLA|>1.2` drops from 39%. Linear-CAN p90 of 625 means some
   corners will still sit on 600 if the plant is linear.
2. Rate-limited corners: mean `|CAN|` of railed `|dLA|>1.2` frames. If it
   stays ~323, the ceiling never arrived and 600 will look like a no-op.
3. Left-hander offset. If +0.35 m becomes +0.5 m, revert the envelope and
   treat cut as a path/FF problem.
4. 0–10 km/h sawtooth. Should match `000001a4` (still 409). If it gets
   worse, the schedule ramp-in at 2.8–4.0 m/s is too early.
5. 63–76 km/h: still 0% rail, plant/model HP ratio still ~1.0. 600 is
   already off by 17 m/s; any new weave there is not this change.
6. Confirm applied `laf` is 5.37 in the 14–54 band and 3.66 elsewhere
   (`lateralTuneStateSP`). If laf stays 3.66 at STEER_MAX 600, stop and
   flash/rebuild — that is the 500 bug again.
7. Confirm applied `lat_delay` ≈ 0.5585 (0.4585 live + the 0.1 profile offset).
   `get_lat_delay` was inverted until 2026-08-27, so this drive is the first on
   the live value — a jerk-keyed change vs `000001a4` may be this, not the 600.

### 2026-08-27 — get_lat_delay was inverted (fixed, affects the next drive)

`livedelay/helpers.py` had both branches of `get_lat_delay` swapped: the
2026-08-24 merge revert (`b77bcd0290`) reversed upstream's own fix (#1906,
`53e13a7bc0`) back into the fork. With `LagdToggle` at its default (on), the
tune was reading the stale `LagdValueCache` param instead of the live lagd
value.

This matters because `lat_delay` is the denominator of `raw_lateral_jerk`, and
`167e6e75` deliberately adds `+0.1` to reproduce StarPilot's
`LAT_SMOOTH_SECONDS` against a measured `lateralDelay` of 0.4585 s. Restored to
upstream and pinned with a test (`livedelay/tests/test_helpers.py`).

**The 600 drive is therefore not a clean A/B against `000001a4`:** it changes
the envelope *and* the lat_delay source. Log the applied `lat_delay` and check
it against 0.4585 before reading anything into a jerk-keyed difference.

## Lateral changes that are NOT behind the v2/StarPilot gate

Everything else is selected by `TorqueControlTune = 2` + `EnforceTorqueControl`.
This one is not, and stays live if you switch the dropdown back to v0:

- **`torqued.py` `MIN_FRICTION_CEILING = 0.10`** floors the multiplicative
  friction sanity ceiling. Without it the Ioniq 6 / EV6 offline friction of
  0.005 collapses the ceiling to 0.0075–0.01 and torqued clips 100% of its
  estimates. Irrelevant while `use_live_torque_params = False`, but it changes
  what torqued publishes on any tune.

### 2026-08-27 — ripple prefilter and turn_intent moved behind the profile

Both used to run wider than this tune. The prefilter lived in
`latcontrol_torque_v2.py`'s shared `update()` (every car selecting v2 got it);
`turn_intent` ran in `controlsd.py` on **every** tune including v0.

Neither is car-agnostic in practice — the 0.78 Hz ripple was measured on this
car's model output, and `turn_intent` was ported from StarPilot's controlsd
alongside the rest of the tune. Now:

- Prefilter → `ioniq6_starpilot.py` via the new `filter_desired_curvature()`
  base hook. v2 applies no shaping of its own; with no profile it is v0.
- `turn_intent/` → `lateral_tunes/turn_intent/`, gated on `uses_turn_intent_hold`.
  It applies upstream of `clip_curvature` so it cannot be a profile method;
  controlsd reads the flag off the live controller, so the hot-swap carries it.

Switching tunes mid-hold now resets the held state instead of leaving it wound.

**Confound for the next drive:** `turn_intent` no longer runs on v0, so a v0
A/B is no longer the same v0 that drove `000001a4`. It is blinker-gated either
way, so hands-off comparisons are unaffected.

## Routes

Dongle `568c82e1de7c61a2`. Rlogs on the Linux box under
`/home/hendrik/openpilot/rlog_data/`.

| route | what was running | used for |
|---|---|---|
| `00000191`, `00000193` | stock 270 + early scheduled 409 | ceiling vs rate-cap by speed |
| `00000194` | rate schedule A/B vs 193 | 54–70 km/h rate sat 44% → 3% |
| `0000019b` | StarPilot 409, saturation by speed | original 500 breakpoint argument |
| `000001a1` | 500 + laf 3.66, v2 PID 1.0/0.3 | killed the 500 experiment |
| `000001a4` | 409 + StarPilot PID 0.6/0.35 | torque deficit in corners; cut is path |

Analysis scripts (untracked): `rlog_data/000001a1/reanalyze_1a1.py`,
`rlog_data/000001a4/analyze_1a4.py`.

## Code map

| piece | where |
|---|---|
| Shaping (pure) | `lateral_tunes/ioniq6_shaping.py` |
| Profile / inner loop | `lateral_tunes/ioniq6_starpilot.py` |
| Profile registry | `lateral_tunes/__init__.py` |
| v2 wrapper (no shaping of its own) | `latcontrol_torque_v2.py` |
| Ripple prefilter | `lateral_tunes/ioniq6_starpilot.py` |
| Turn-intent hold | `lateral_tunes/turn_intent/` |
| CAN envelope + rate | `opendbc/sunnypilot/car/hyundai/lateral_limits.py` |
| Tune flag / baseline | `opendbc/sunnypilot/car/hyundai/values.py` |
| Panda envelope | `opendbc/safety/modes/hyundai_canfd.h` |
| Regression | `lateral_tunes/tests/test_lateral_tunes.py` |
