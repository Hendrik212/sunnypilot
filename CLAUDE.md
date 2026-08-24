## Pre-deploy checks (mandatory)

Run these before pushing anything that the device will run. Skipping them broke a drive
on 2026-08-24 (see "Post-mortem" below).

1. **Run the repo's own lint, unfiltered.** `uvx ruff check openpilot/ opendbc_repo/opendbc/`
   `pyproject.toml` already sets `lint.select = ["E", "F", ...]`, and `F` includes **F821
   (undefined name)** — which catches the single most likely refactor defect: a local that
   stopped being in scope. Do **not** pipe lint output through `head`/`tail`; read the
   summary line and the whole list, or you will cut off the error that matters.
2. **Remember that offroad verification does not exercise engaged code paths.** Anything
   inside `if active:` / `latActive` branches — the whole lateral control body — is never
   touched by an offroad params/constants check. A clean on-device readout of
   `STEER_MAX` / `latAccelFactor` says nothing about whether `update()` survives engaging.
3. **After a refactor that splits or moves a function, diff the parameter lists.** Names
   that were locals of the original function are the failure mode; the call still parses,
   so only F821 or a real engaged run finds it.
4. **`git merge` can silently drop a fork-local fix** when upstream touches the same file.
   After merging, re-check the specific lines your fork fixed, not just that the merge
   was conflict-free.

## Route/log analysis stack

Route analysis (torque tuning, lateral ping-pong, rate-limit violations) is done on the
Linux dev box, not on this Windows machine — capnp/cereal tooling doesn't run here.

- Host: `192.168.1.131` (user `hendrik`) — see `ENVIRONMENT.md` for full host details.
- openpilot checkout: `~/openpilot` (fork `Hendrik212/openpilot`, branch `isla-master`).
- Staged rlog data: `~/rlog_data/` (segment dirs, each with `rlog`/`rlog.zst`).

### Gen-2 (current): Cereal MCP server
- `~/openpilot/tools/mcp/cereal_mcp.py` — stdio MCP server, mirrors the `tuning.xml`
  PlotJuggler layout. Tools: `list_segments`, `get_lateral_data`, `get_signal_values`,
  `analyze_lateral` (ping-pong/oscillation detection with a model-vs-controls diagnosis
  hint). Documented in `tools/mcp/README.md` in this repo.
- Committed here as of `37121d66ba`; the copy on the Linux box may be untracked/ahead —
  diff before assuming they match.

### Gen-1 (older): direct capnp extraction scripts
- On the Linux box only, untracked: `~/openpilot/extract_logs.py`,
  `~/openpilot/tools/extract_torque_logs.py`, `~/openpilot/tools/analyze_lkas_error.py`,
  `~/openpilot/tools/check_steering_method.py`.
- Writeups (also untracked, Linux box only): `~/openpilot/TORQUE_DYNAMIC_LIMITS.md`,
  `~/openpilot/TORQUE_FIX_SUMMARY.md`, `~/openpilot/CAN_FINDINGS.md` — includes the
  methodology (transfer → engagement segmentation → signal extraction → rate-limit
  checks → per-engagement summary) and the history of a prior high-STEER_MAX wobble
  episode on the Ioniq 6.

### Fetching a route from comma connect

Routes the user has marked **public** are downloadable with no auth token (there is no
`~/.comma/auth.json` on the dev box):

```bash
R="568c82e1de7c61a2|00000196--25611b1b00"          # dongle_id|route  (pipe, not slash)
curl -sS "https://api.commadotai.com/v1/route/${R}/files" -o files.json
python3 -c "import json;print('\n'.join(json.load(open('files.json'))['logs']))" > urls.txt
i=0; while read -r u; do curl -sS -o "rlog$i.zst" "$u"; i=$((i+1)); done < urls.txt
```

`files.json` has `logs` (one `rlog.zst` per segment), plus `cameras`/`qcameras` when
uploaded. The blob URLs carry a **SAS token that expires in minutes** — if a download
returns 0 bytes, re-fetch `files.json` and retry that segment.

### Diagnosing a crash from a route

Decode with the minimal venv (see the analysis-stack notes; system Python lacks the deps):

```python
import zstandard, capnp
capnp.remove_import_hook()
log = capnp.load("openpilot/cereal/log.capnp",
                 imports=["opendbc_repo/opendbc/car", "openpilot/cereal"])
raw = zstandard.ZstdDecompressor().stream_reader(open("rlog0.zst","rb")).read()
msgs = list(log.Event.read_multiple_bytes(raw))
```

Read in this order — it goes from symptom to traceback fast:

1. `collections.Counter(m.which() for m in msgs)` — which services are publishing at all.
2. `onroadEvents` — the alerts the user actually saw (`e.name` is an enum; `str()` it
   before formatting or `%s`-style formatting raises).
3. `errorLogMessage` / `logMessage` — JSON payloads; `d["ctx"]["daemon"]` names the
   process, and a crashed Python process logs its **full traceback** here.
4. `managerState[-1].processes` — `running` / `exitCode` per process.
5. `selfdriveState.enabled` transitions — pin down the exact engage moment.

**Service names in this fork are not upstream's.** Check the Counter output before
concluding something is missing: it logs `liveLocationKalman` (not `livePose`),
`extrinsicsCalibration` (not `liveCalibration`), `lateralTorqueParameters` (not
`liveTorqueParameters`), and `radarTracks` (not `liveTracks`).

### Workflow
1. Pull rlogs from the device (`ioniq_local`, 192.168.1.197) to the Linux box.
2. Run the extraction/analysis tooling above from `~/openpilot` on the Linux box.

## Future integration ideas

- **Ioniq 5 CAN reverse-engineering**: https://github.com/tylerharvey/Ioniq5_CAN.git — not yet
  investigated in depth, but likely applicable to the Ioniq 6 too (shared E-GMP platform/CAN
  bus) and should translate to openpilot/opendbc the same way other community DBC work has.
  Worth a look next time we're back in the Hyundai CAN weeds.

## Post-mortem: 2026-08-24 controlsd crash on engage

Worth re-reading before the next lateral refactor — three wrong diagnoses preceded the
right one, and the route log settled it in minutes.

- **Symptom:** stock LKAS deactivated normally at boot, then on engage the UI showed a
  communication error + "process not running" and the cluster lit up.
- **Cause:** `NameError: name 'calibrated_pose' is not defined` in
  `latcontrol_torque_v2.py::_update_ioniq6`. Splitting `update()` into
  `_update_upstream()` / `_update_ioniq6()` moved the `self.extension.update(...)` call
  into the helpers but left `calibrated_pose` a local of `update()`. Both v2 paths were
  broken. controlsd crashed 0.1 s after engaging, every time.
- **Why it escaped:** the reference is only reachable when `active`. Import-time checks,
  `py_compile`, the panda safety suite and an on-device constants readout all passed.
- **Wrong turns worth not repeating:**
  - Blamed an upstream model-index bump (`driving_models_v19.json` → `v20`). Disproved by
    diffing the two JSONs — identical bundle sets, same `minimumSelectorVersion`.
  - Blamed the Hyundai radar cherry-pick. Disproved by the route: `radarTracks` 2652 msgs,
    `radarState` 1065. A local repro used `gen_empty_fingerprint()`, which returns no radar
    tracks and does not represent the real car — **use a real fingerprint when reproducing
    car-specific behaviour.**
  - Reverted the upstream merge as a safety net. It did not help: the bug predated the
    merge, so the revert restored a tree containing the same crash.
- **Lesson:** when the device fails, fetch the route before forming a theory. Static
  reasoning produced three plausible, confident, wrong answers; `onroadEvents` +
  `errorLogMessage` produced the traceback directly.
