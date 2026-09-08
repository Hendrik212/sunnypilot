# Upstream Merge Guide for Sunnypilot ISLA Fork

This guide documents the process for merging upstream changes from the main sunnypilot repository and submodules into the ISLA fork while preserving custom modifications.

> **Directory layout note:** as of the 2026-07 upstream restructure, nearly the entire
> sunnypilot tree (`common/`, `selfdrive/`, `system/`, `sunnypilot/`, `cereal/`, etc.) now
> lives under `openpilot/`. Only submodules (`opendbc_repo`, `panda`, `msgq_repo`,
> `rednose_repo`, `teleoprtc_repo`, `tinygrad_repo`) and a handful of meta-repo dirs
> (`docs/`, `release/`, `scripts/`, `site_scons/`, `tools/`, plus our own `pyextra/`) stay
> at the true top level. All paths below reflect this.

## Repository Structure

| Repo | Origin (our fork) | Upstream | Our Branch |
|------|-------------------|----------|------------|
| sunnypilot | `Hendrik212/sunnypilot` | `sunnypilot/sunnypilot` | `isla-master` |
| opendbc | `Hendrik212/opendbc` | `sunnypilot/opendbc` | `sp-isla-master` |
| panda | `Hendrik212/panda` | `sunnyhaibin/panda` | `sunnypilot-master` |

## Prerequisites

- Ensure you have proper remotes configured:
  ```bash
  cd D:\dev\openpilot\sunnypilot
  git remote -v
  # Should show:
  # origin    https://github.com/Hendrik212/sunnypilot.git (fetch/push)
  # upstream  https://github.com/sunnypilot/sunnypilot.git (fetch/push)
  ```

- If upstream remote doesn't exist, add it:
  ```bash
  git remote add upstream https://github.com/sunnypilot/sunnypilot.git
  ```

- Verify opendbc submodule remotes:
  ```bash
  cd opendbc_repo
  git remote -v
  # Should show:
  # origin    https://github.com/Hendrik212/opendbc.git (fetch/push)
  # upstream  https://github.com/sunnypilot/opendbc.git (fetch/push)
  ```

## Step-by-Step Merge Process

### 1. Prepare for Merge

```bash
# Ensure you're on the correct branch
git checkout isla-master

# Check current status — should be clean
git status
```

### 2. Fetch Latest Changes

```bash
git fetch upstream
git fetch origin
```

### 3. Merge Upstream Sunnypilot

```bash
git merge upstream/master
```

**If merge succeeds without conflicts:**
- Skip to step 5 (Handle Submodule Updates)

**If conflicts occur:**
- Continue to step 4

### 4. Resolve Conflicts

Files most likely to conflict with our custom changes:

- `openpilot/system/manager/process_config.py` — our MQTT, BLE process entries
- `openpilot/system/hardware/power_monitoring.py` — our shutdown logic
- `openpilot/selfdrive/selfdrived/events.py` — our steerSaturated + speedTooHigh silencing
- `openpilot/cereal/log.capnp` — our MQTT message types (`mqttPubQueue`, `mqttRecvQueue`)
- `openpilot/cereal/services.py` — our MQTT service entries

Custom-only directories that don't exist upstream at all — git has no upstream content to
map them against, so on a big upstream directory rename they can get silently left behind
at their old path instead of following the rest of the tree. After any merge that moves
files around, verify these still live under `openpilot/system/`:

- `openpilot/system/mqttd/` — MQTT daemon (entirely custom)
- `openpilot/system/abrp_ble/` — ABRP BLE bridge (entirely custom)

(`pyextra/` stays at the true top level, not under `openpilot/` — it's a Python package
root referenced as `from pyextra.paho...`, independent of the `openpilot.` import prefix.)

```bash
# Resolve conflicts manually in each file
# Keep our custom additions, accept upstream changes elsewhere
git add <resolved-files>
```

> **⚠️ Directory-rename-divergence conflicts:** on a large upstream restructure (files
> moved/renamed en masse), git may flag files that exist on only one side of the merge
> with a message like *"file added in HEAD inside a directory that was renamed in
> upstream/master, suggesting it should perhaps be moved to \<new-path\>"* (status `AU`/`D`
> pair, not `UU`). This happens for anything that entered our history through a *previous*
> merge but isn't in upstream's current tree — either a feature upstream later reverted, or
> genuinely custom content. It is NOT usually a content conflict (0 conflict markers in the
> file) — it's git asking where to place it. Check whether the file/feature still works
> (imports resolve, no orphaned references in SConscript/other files) before accepting git's
> suggested new path with `git add <new-path>`. Don't reflexively delete just because
> upstream dropped it — if it's self-contained and was already working, keep it.

> **⚠️ capnp ordinal gotcha (`openpilot/cereal/log.capnp`):** capnp requires the `Event` struct's
> ordinals to be **sequential with no holes**, and our `mqttPubQueue`/`mqttRecvQueue` must
> occupy the **highest** ordinals. A text merge will NOT flag a conflict, but if upstream
> added a new `Event` field it will reuse the ordinal our mqtt fields had, producing a
> *duplicate ordinal* error at runtime (capnp import crash — surfaces when anything imports
> `cereal`, e.g. clearing params). After every merge, bump our two mqtt fields to the next
> sequential ordinals above upstream's new max. Do **not** jump to a high "reserved" number —
> that creates a hole and capnp rejects it ("Skipped ordinal @N"). Verify with a `cereal`
> import on-device before rebooting.

### 5. Handle Submodule Updates

The opendbc submodule is the most important — it has our Ioniq 6 longitudinal changes.

```bash
cd opendbc_repo

# Fetch BOTH remotes — origin can carry commits merged directly on GitHub that your
# local clone doesn't have (this has happened — see note below). Don't assume local
# HEAD == origin HEAD just because you haven't pushed anything yourself recently.
git fetch upstream
git fetch origin

# Merge upstream into our branch
git checkout sp-isla-master
git merge upstream/master

# If conflicts occur, resolve them — our custom files:
#   opendbc/car/hyundai/carcontroller.py — cancel timeout, standstill resume
#   opendbc/car/hyundai/carstate.py — BSM disable during long, conditional CAN parser
#   opendbc/car/hyundai/hyundaicanfd.py — ACCMode 0 in cancel
#   opendbc/car/hyundai/interface.py — BSM address, ECU silence verification
#   opendbc/car/hyundai/values.py — Ioniq 6 flags (no CANFD_NO_RADAR_DISABLE), steer limits
#   opendbc/car/hyundai/mqtt.py — MQTT CAN data parser (entirely custom)
#   opendbc/car/disable_ecu.py — verify_silence_addrs support
#   opendbc/car/hyundai/radar_interface.py — see radar-tracks note below
#   opendbc/sunnypilot/car/hyundai/radar_interface_ext.py — see radar-tracks note below

git add <resolved-files>
git commit -m "Merge upstream sunnypilot/opendbc into sp-isla-master"

# If origin had commits you didn't have locally, merge those in too before pushing:
git merge origin/sp-isla-master --no-edit

# Push opendbc submodule
git push origin sp-isla-master

cd ..
```

> **Radar tracks (`opendbc/car/hyundai/radar_interface.py` + `radar_interface_ext.py`):**
> as of 2026-07-20 these are pulled from sunnypilot's `upstream/hyundai-radar-tracks` topic
> branch (not `upstream/master` — that branch's own history diverged from ours ~110
> commits back and hasn't tracked master since). Merge it explicitly if you want its
> updates:
> ```bash
> git fetch upstream hyundai-radar-tracks
> git merge upstream/hyundai-radar-tracks --no-edit
> ```
> This branch is a full rewrite: runtime CAN-fingerprint auto-detection across 5 radar
> specs (`RADAR_500_53F/210_21F/235_248/3A5_3C4/602_617`, our Ioniq 6 = `3A5_3C4`) replacing
> the old static per-platform `MRR35_RADAR`-style flags entirely — those flags no longer
> exist in `HyundaiFlags`, so don't reference them in platform configs anymore (`HYUNDAI_IONIQ_6`
> is just `HyundaiFlags.EV` now). It also added new `car.capnp` fields (`RadarData.trackSources`,
> `radarTracksAvailable`, `RadarPoint.motionState/sourceAddress/sourceBus/trackAge`) and
> un-deprecated `aRel`/`yvRel`/`measured` (same ordinals). The `.dbc` files these specs need
> are generator-produced at build time (gitignored, not checked in — verify with
> `python -c "from opendbc.dbc.generator.generator import generate_all; ..."` if paranoid).
>
> ⚠️ It's still WIP (messy commit history, hasn't landed on `sunnypilot/opendbc:master`) —
> treat every merge from it as a real review, not a rubber-stamp. And critically: radar mode
> is gated by a user param (`"RadarTracks"`, read in `opendbc/sunnypilot/car/interfaces.py`
> `_initialize_radar()`), defaulting to `RadarType.OFF`. Our Ioniq 6 has no `CAMERA_SCC`/`ESCC`,
> so `RadarType.LEAD_ONLY` is a no-op for us — we need `FULL_RADAR` (2), which the existing UI
> toggle (`radar_tracks` in `toggles.py`, a plain boolean `BigParamControl`) can't reach. Check
> whether that's been resolved before assuming radar "just works" post-merge.

If panda submodule was also updated by upstream:
```bash
cd panda
git fetch upstream
git fetch origin
git checkout sunnypilot-master
git merge upstream/master
git merge origin/sunnypilot-master --no-edit   # pick up any commits pushed directly to origin
git push origin sunnypilot-master
cd ..
```

### 6. Update Submodule Pins and Commit

```bash
# Stage the updated submodule references
git add opendbc_repo panda

# Complete the merge commit
git commit -m "Merge upstream sunnypilot/master into isla-master"

# Push (skip LFS — objects are on commaai's server)
GIT_LFS_SKIP_PUSH=1 git push origin isla-master
```

## Custom Modifications to Preserve

### Main Repository (sunnypilot)

| File | Change |
|------|--------|
| `openpilot/system/manager/process_config.py` | MQTT + BLE processes, ubloxd/pigeond disabled |
| `openpilot/system/hardware/power_monitoring.py` | Relaxed shutdown (11V floor only) |
| `openpilot/selfdrive/selfdrived/events.py` | steerSaturated silenced (both instances), speedTooHigh silenced |
| `openpilot/cereal/log.capnp` | MqttPubQueue, MqttRecvQueue structs — ordinals bump every merge, see gotcha below |
| `openpilot/cereal/services.py` | mqttPubQueue, mqttRecvQueue service entries |
| `openpilot/system/mqttd/` | Entire directory (custom) |
| `openpilot/system/abrp_ble/` | Entire directory (custom) |
| `pyextra/paho/` | Bundled paho-mqtt library (stays top-level, not under `openpilot/`) |

### opendbc Submodule

| File | Change |
|------|--------|
| `opendbc/car/hyundai/values.py` | Ioniq 6: removed `CANFD_NO_RADAR_DISABLE`. (CANFD steer limits are currently **stock** — 270/2/3 — see note below if re-tuning) |
| `opendbc/car/hyundai/carcontroller.py` | Cancel timeout (4s), standstill resume fix |
| `opendbc/car/hyundai/carstate.py` | BSM disabled during long, conditional CAN parser |
| `opendbc/car/hyundai/hyundaicanfd.py` | ACCMode 0 in create_acc_cancel |
| `opendbc/car/hyundai/interface.py` | BSM address 0x1ba, ECU silence verification |
| `opendbc/car/hyundai/mqtt.py` | Ioniq 6 CAN data parser (entirely custom) |
| `opendbc/car/disable_ecu.py` | verify_silence_addrs, _verify_ecu_silence |
| `opendbc/car/hyundai/radar_interface.py` | Pulled from `upstream/hyundai-radar-tracks` topic branch, not `master` — see radar-tracks note in Step 5 |
| `opendbc/sunnypilot/car/hyundai/radar_interface_ext.py` | Same — pulled from the radar-tracks branch, kept in sync with radar_interface.py |

### modeld / Chestnut big model (sunnypilot main repo)

The chestnut split-warp compile path is entirely fork-local. Upstream does not have
`--split-warp`, `WARP_DEV`, or the re-chunk fix. A conflict-free merge of upstream's
`SConscript` or `compile_modeld.py` **will silently drop these and reintroduce the
bugs they fix** — so after any merge that touches modeld, re-verify every line below.

| File | Fork-local change | If dropped |
|------|-------------------|-----------|
| `openpilot/selfdrive/modeld/SConscript` | Chestnut branch routes to `sunnypilot/modeld_v2/compile_modeld.py` with `WARP_DEV=QCOM --split-warp` | Build uses the fused `run_model` → 7.47 MB/frame USB transfer → frame drops on comma 3X |
| `openpilot/selfdrive/modeld/SConscript` | `do_compile` `_chestnut` guard skips double-chunking (v2 compiler already chunks) | `No such file or directory` — chunks a deleted `.pkl`, build fails |
| `openpilot/selfdrive/modeld/SConscript` | `do_compile` reassembles + re-chunks to declared targets (commit `02403e55a2`) | **Rebuild-on-every-boot**: v2 writes 17 `chunkNNof17`, scons expects 33 `chunkNNof33` → 16 missing targets → recompile every boot (~10 min, blocks manager) |
| `openpilot/sunnypilot/modeld_v2/compile_modeld.py` | `make_random_model_inputs` creates `warped` on `warp_dev`, not `Device.DEFAULT` | `JitError: args mismatch in JIT` swallowed by `load_big`'s `except` → modeld crash-loops `exitCode=1`, nothing in swaglog |
| `openpilot/sunnypilot/modeld_v2/compile_modeld.py` | `make_split_warp()` + `--split-warp` argparse branch | No split-warp path at all |
| `launch_env.sh` | Sets `COMBINED_MODEL_PKL` unconditionally when `lsusb` shows `3801:0001` | modeld uses the model-manager bundle (small model) instead of the compiled big pkl |
| `openpilot/selfdrive/modeld/dmonitoringmodeld.py` | `config_realtime_process(6, 5)` (was `(7, 5)`) | dmon shares core 7 with modeld → core saturation → ~2 ms slower, more drops |

> **Post-merge checklist for modeld** (do this even if the merge was conflict-free):
> 1. `grep -n 'split-warp\|WARP_DEV\|_chestnut\|open_file_chunked' openpilot/selfdrive/modeld/SConscript` — all four must return hits.
> 2. `grep -n 'device=warp_dev' openpilot/sunnypilot/modeld_v2/compile_modeld.py` — the `warped` JIT-input fix must be present.
> 3. `grep -n 'COMBINED_MODEL_PKL' launch_env.sh` — must set it when chestnut present.
> 4. Boot the device and confirm manager starts **without** a `compile_modeld` process (if it recompiles, the re-chunk fix was dropped).
>
> **Why scons usually won't rebuild the big model on a merge:** `SConstruct` uses
> `Decider('MD5-timestamp')` — mtime check first, and only if mtime changed does it
> compute an MD5 and compare to the cached content hash. So a `git merge` that updates
> a dep file's mtime but not its content → **no rebuild**. Only a real content change
> (new ONNX, changed compiler, moved tinygrad submodule pointer) triggers a recompile.
> This holds as long as `.sconsign.dblite` persists (it does, in the repo dir on device).
> A merge that touches `SConscript` itself *can* invalidate the target definition
> regardless of content — so expect one rebuild after a SConscript-touching merge, then
> none. See [[chestnut-model-compile-workflow]] for the full gotcha list.

### panda Submodule

| File | Change |
|------|--------|
| `board/main_comms.h` | `heartbeat_engaged_mads` forced `true` (was `req->param2 == 1U`) — works around a startup race where `controls_allowed_lateral` drops if `selfdriveStateSP` isn't alive yet when pandad's first heartbeat lands, causing CAN errors on every steer command until reboot. TODO-SP marked in code; revert once the startup race is fixed upstream. |

> **Steer-limit tuning (currently at stock).** The CAN-FD steer limits are presently unmodified from upstream, so `hyundai_canfd.h` is not in the table above. **If you re-tune them**, `values.py` and `hyundai_canfd.h` must always be kept in sync. The panda safety layer (`hyundai_canfd.h`) enforces hard limits in firmware — if `values.py` requests more torque or a higher rate than the safety code allows, commands will be silently clipped or trigger a safety fault. Whenever you change `STEER_MAX`, `STEER_DELTA_UP`, or `STEER_DELTA_DOWN` in `values.py`, update `max_torque`, `max_rate_up`, and `max_rate_down` in the `HYUNDAI_CANFD_STEERING_LIMITS` struct accordingly.
>
> **You must also recalculate `max_rt_delta`** when changing `max_rate_up`. This is the maximum total torque change allowed within a 250ms real-time window (defined by `MAX_RT_INTERVAL` in `declarations.h`). The formula is:
>
> ```
> max_rt_delta = max_rate_up * (250ms / STEER_STEP_period) * margin
>             = max_rate_up * 25 * 1.12
> ```
>
> At 100Hz (STEER_STEP=1, 10ms per frame), there are 25 frames per 250ms window. The 12% margin prevents false RT violations. If `max_rt_delta` is too low for the configured `max_rate_up`, the safety layer will reject valid torque ramps and cause EPS faults.
>
> | max_rate_up | max_rt_delta |
> |-------------|--------------|
> | 2           | 56           |
> | 3           | 84           |
> | 4           | 112          |
> | 5           | 140          |
>
> Upstream sunnypilot stock CANFD values for reference: `STEER_MAX=270`, `STEER_DELTA_UP=2`, `STEER_DELTA_DOWN=3`, `max_rt_delta=112`.

## Preparing a New Big Model (Chestnut / comma 3X)

When commaai releases a new driving model (e.g. a successor to Cinque Terre), it ships
as a new `driving_supercombo.onnx` in `openpilot/selfdrive/modeld/models/`. The chestnut
big-model path must compile it with `--split-warp` (warp on QCOM, policy on AMD) — the
fused path ships 7.47 MB raw NV12 per frame over USB and drops frames on the comma 3X's
1928×1208 sensor. This is the procedure that worked for Cinque Terre (2026-09-08).

### 1. Swap in the new ONNX

```bash
cd /mnt/sdc1/openpilot/sunnypilot-isla

# The big model ONNX is LFS-tracked. Fetch from commaai's server.
git fetch commaai master
git checkout commaai/master -- openpilot/selfdrive/modeld/models/big_driving_supercombo.onnx

# Verify it's the right model — record the sha for later comparison
sha256sum openpilot/selfdrive/modeld/models/big_driving_supercombo.onnx
```

> If the ONNX is chunked on disk (LFS may store it as `.onnx.chunk*` + `.chunkmanifest`),
> reassemble first: `cat big_driving_supercombo.onnx.chunk* > big_driving_supercombo.onnx`.

### 2. Confirm the split-warp compile path is intact

Run the post-merge checklist above (step 1–3). The `--split-warp` flag, `WARP_DEV=QCOM`,
the `_chestnut` guard, and the re-chunk fix must all be present in `SConscript`. If a
recent merge dropped any of them, restore from the last known-good commit before compiling.

### 3. Iterate the compile standalone over SSH — NOT via SConscript+reboot

This is the single most important process lesson: **never iterate `compile_modeld.py`
through `SConscript` + reboot cycles.** Each trivial error (wrong path, missing import,
kwarg collision) costs a full 10-min boot *and* blocks manager from starting. Run the
compiler directly over SSH and fix errors in seconds:

```bash
ssh ioniq_local 'sudo systemctl stop comma.service; sleep 4
  cd /data/openpilot && GMMU=0 PYTHONPATH=/data/openpilot DEV=USB+AMD:LLVM WARP_DEV=QCOM \
  FLOAT16=1 JIT_BATCH_SIZE=0 TC_OPT=2 TC_OCCUPANCY_OPT=1 \
  taskset -c 7 /usr/local/venv/bin/python3 openpilot/sunnypilot/modeld_v2/compile_modeld.py \
    --model-type supercombo --model-size 512x256 --camera-resolutions 1928x1208 \
    --supercombo-onnx openpilot/selfdrive/modeld/models/big_driving_supercombo.onnx \
    --split-warp --output /tmp/test.pkl --frame-skip 4'
```

Only wire it into SConscript once it works end to end. See [[chestnut-model-compile-workflow]]
for why and the full gotcha list.

> **Cannot cross-compile on the dev box.** The dev box has gfx1100 (RDNA3) and no Adreno;
> the chestnut eGPU is GFX12/RDNA4 and the warp runs on QCOM Adreno 690. tinygrad's JIT
> emits device-specific ISA and captures real GPU command buffers, so the pkl is not
> portable across GPU archs. Compile on the device. (pkls *are* portable across identical
> hardware — sunnypilot's model manager downloads precompiled pkls from HuggingFace — but
> our dev box is not identical to the comma 3X+chestnut.)

### 4. The one non-obvious bug to watch for

`make_random_model_inputs` for `run_policy` must create the `warped` tensor on
**`warp_dev`** (QCOM), not `Device.DEFAULT` (AMD). `warped` is the warp JIT's *output*,
so at runtime it arrives on QCOM. TinyJit validates input device signatures at
`jit.py:280` *before* the function body runs, so `run_policy`'s own
`warped.to(Device.DEFAULT)` never gets a chance. The failure is a bare
`JitError: args mismatch in JIT` that `load_big`'s `except` swallows — modeld crash-loops
with `exitCode=1` and **nothing in swaglog**. If modeld dies silently after a model
compile, this is the first thing to check.

### 5. Deploy and verify onroad

Once the standalone compile succeeds, push the new ONNX + any compiler fixes, then let
SConscript compile the real pkl on the next boot:

```bash
# On the dev box:
GIT_LFS_SKIP_PUSH=1 git push origin isla-master

# On the device:
cd /data/openpilot && git pull --ff-only origin isla-master
sudo systemctl restart comma.service
# Wait ~10 min for the one-time compile, then verify:
```

Verify the result (the staged script lives at `/data/measure_exec.py` on the device):

```bash
ssh ioniq_local 'cd /data/openpilot && PYTHONPATH=/data/openpilot/openpilot \
  /usr/local/venv/bin/python3 /data/measure_exec.py'
```

Expected for a healthy split-warp big model on comma 3X:
- `modelExecutionTime` p50 ≈ **33 ms**, p99 < 36 ms, std < 1 ms
- `P(>50ms) = 0.00%`
- `frameDropPerc` p50 = 0.00, max = 0.00 (engage blocks when > 1.0)
- `big model = 100%` of frames (no silent fallback to small model)

Baseline (fused, for comparison): p50 49.2, p99 52.0, P(>50ms) 22.7%, drops 4–6%.

### 6. Back up the working pkl

Once verified, back up the chunks immediately — two prior attempts were lost to rebuilds
that deleted chunks mid-copy:

```bash
mkdir -p /mnt/sdc1/openpilot/model_backups/<model-name>-splitwarp-<date>
scp ioniq_local:'/data/openpilot/openpilot/selfdrive/modeld/models/big_driving_tinygrad.pkl.chunk*' \
  /mnt/sdc1/openpilot/model_backups/<model-name>-splitwarp-<date>/
# Verify byte-identical:
sha_device=$(ssh ioniq_local 'cd /data/openpilot/openpilot/selfdrive/modeld/models && cat big_driving_tinygrad.pkl.chunk*of* | sha256sum')
sha_local=$(cat /mnt/sdc1/openpilot/model_backups/<model-name>-splitwarp-<date>/big_driving_tinygrad.pkl.chunk*of* | sha256sum)
[ "$sha_device" = "$sha_local" ] && echo "backup verified" || echo "MISMATCH"
```

### 7. Confirm no rebuild on subsequent boots

After the one-time compile, restart the service once more and confirm `compile_modeld`
does **not** run (manager should come up directly within ~30 s). If it recompiles again,
the re-chunk fix (`02403e55a2`) was dropped or the chunk targets don't match — see the
modeld post-merge checklist above.

## Git LFS Handling

The repository uses Git LFS for large files (models, binaries). CommaAI's LFS server hosts these files, but we don't have write access.

**Always use `GIT_LFS_SKIP_PUSH=1` when pushing:**
```bash
GIT_LFS_SKIP_PUSH=1 git push origin isla-master
```

This works because `.gitattributes` points LFS to CommaAI's server — clones automatically fetch models from there.

## Deploying to Device

```bash
# SSH to comma device
ssh comma@192.168.1.197  # or ioniq_local

# Pull and update
cd /data/openpilot
git pull origin isla-master
git submodule update --init
# ⚠️ Do NOT scope this to opendbc_repo only. A merge that bumps ANY submodule pin
# (msgq_repo, teleoprtc_repo, tinygrad_repo, panda) needs ALL of them synced — an
# opendbc-only update once left msgq_repo stale after upstream restructured its
# headers, breaking the on-device build (missing msm_ion.h).

# Clear stale car params if opendbc flags changed
PYTHONPATH=/data/openpilot /usr/local/venv/bin/python3 -c "
from openpilot.common.params import Params
Params().remove('CarParamsPersistent')
Params().remove('CarParamsCache')
"

# Reboot
sudo reboot
```

**Important:** If you changed Ioniq 6 flags in values.py, you MUST clear `CarParamsPersistent` — otherwise the device uses cached flags and your changes won't take effect.

## Emergency Recovery

```bash
# Abort ongoing merge
git merge --abort

# Reset to last known good state
git reset --hard HEAD~1

# Or reset to specific commit
git reset --hard <commit-hash>
```

## Troubleshooting

### "Not a git repository" in Submodule
```bash
git submodule update --init --recursive
```

### Submodule Detached HEAD
```bash
cd opendbc_repo
git checkout sp-isla-master
cd ..
```

### Device Shows Old Flags / No Long Toggle
Clear cached car params (see Deploying to Device section above) and reboot with ignition on.

### commIssue After Merge
Usually caused by cereal schema mismatch. Device needs a full rebuild — reboot and wait for `build.py` to complete before driving.
