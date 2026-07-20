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
#   opendbc/car/hyundai/radar_interface.py — differentiated MRR30/MRR35/MRREVO14F parsing
#   opendbc/sunnypilot/car/hyundai/radar_interface_ext.py — RADAR_LEAD_ONLY/FULL_RADAR/OFF flags
#     ⚠️ upstream/master's radar_interface.py only implements the generic Mando radar
#     (single AZIMUTH-based parser). Our Ioniq 6 uses MRR35_RADAR, whose DBC
#     (hyundai_mrr35_radar_generated) has NO AZIMUTH signal — taking upstream's version
#     verbatim will KeyError-crash radar parsing on-device. Always keep our differentiated
#     version here unless upstream has genuinely added equivalent per-radar-type support
#     (check by diffing merge-base→upstream/master for this file specifically, not just
#     resolving the conflict hunk — small unrelated upstream cleanups, e.g. dropping
#     deprecated RadarPoint fields, are fine to skip/ignore).

git add <resolved-files>
git commit -m "Merge upstream sunnypilot/opendbc into sp-isla-master"

# If origin had commits you didn't have locally, merge those in too before pushing:
git merge origin/sp-isla-master --no-edit

# Push opendbc submodule
git push origin sp-isla-master

cd ..
```

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
| `opendbc/car/hyundai/radar_interface.py` | Differentiated MRR30/MRR35/MRREVO14F radar parsing (upstream only has generic Mando/AZIMUTH parser — see conflict-resolution note in Step 5) |
| `opendbc/sunnypilot/car/hyundai/radar_interface_ext.py` | RADAR_LEAD_ONLY/RADAR_FULL_RADAR/RADAR_OFF flag scaffolding (`HyundaiFlagsSP`, defined but not yet assigned to any platform — dead code path for now, keep anyway since it ships with radar_interface.py) |

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
