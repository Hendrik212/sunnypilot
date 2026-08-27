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

Pushed and driven: **409 flat + StarPilot PID** (`9f2d11fd` / opendbc
`5923cfef`). Last driven route: `000001a4` on `6024dfa88`.

Local, not committed, not driven: **speed-scheduled 600 envelope with
proportional `latAccelFactor`**. Peak panda `max_torque = 600`. Unsaturated
mapping held at `409/3.66 ≈ 112` CAN per m/s².

```
v (m/s)     km/h      STEER_MAX   latAccelFactor   CAN / m/s²
≤ 2.8        ≤ 10          409            3.66            112
4.0–15.0    14–54          600            5.37            112
≥ 17.0       ≥ 61          409            3.66            112
```

PID stays `KP=0.6 / KI=0.35`. Rate stays 10/8 below 17 m/s, ramping to 2/3 by
19.4 m/s (70 km/h). Live torque params remain off. Stock CANFD (tune off) stays
270/2/3.

A panda rebuild/flash is required before the 600 drive. If firmware stays at
409, commands above 409 are silently dropped.

## Lessons that keep being expensive

These are the ones we have now paid for more than once.

1. **`STEER_MAX` without scaling `latAccelFactor` is a gain change, not
   headroom.** `carcontroller` does `CAN = torque * STEER_MAX` and
   `torque ≈ lataccel / latAccelFactor`. Raising only the ceiling raises CAN
   per m/s² on every unsaturated command. That was the 500 experiment.
2. **Panda `max_torque` must equal the peak car-layer request.** A mismatch
   clips with no alert. Tests in `test_lateral_tunes.py` read
   `hyundai_canfd.h` and walk 0–40 m/s.
3. **`max_rt_delta` must cover `max_rate * 25` frames in 250 ms.** Raising
   rate to 10 without moving `max_rt_delta` off 112 was the 720/500 wobble
   (`a6095657`, `95dc60fe`): the RT check rejected the message, reset
   `desired_torque_last` to 0, and the wheel dropout-ramped. Current
   `max_rt_delta = 375`.
4. **Creep-band railing is P/LSF bang-bang, not missing torque.** A taller
   rail there is a bigger sawtooth. 0–10 km/h stays 409 on purpose.
5. **Desired-path cut toward oncoming is not a torque-authority bug.**
   Left-handers on `000001a4` sat 35 cm left of center with `a/d ≈ 1` and 0%
   rail. More peak torque can make that worse.
6. **Offroad / host-boot / constants readout never execute `if active:`.**
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
StarPilot `latcontrol_vehicle_tunes.py` into `latcontrol_ioniq6_tune.py`,
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

550 was considered first; p90=625 made 600 the next ceiling. Uncommitted.

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
| Shaping (pure) | `latcontrol_ioniq6_tune.py` |
| Profile / inner loop | `lateral_tunes/ioniq6_starpilot.py` |
| Profile registry | `lateral_tunes/__init__.py` |
| v2 wrapper + ripple filter | `latcontrol_torque_v2.py` |
| CAN envelope + rate | `opendbc/sunnypilot/car/hyundai/lateral_limits.py` |
| Tune flag / baseline | `opendbc/sunnypilot/car/hyundai/values.py` |
| Panda envelope | `opendbc/safety/modes/hyundai_canfd.h` |
| Regression | `lateral_tunes/tests/test_lateral_tunes.py` |
