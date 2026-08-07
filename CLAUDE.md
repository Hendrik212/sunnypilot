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

### Workflow
1. Pull rlogs from the device (`ioniq_local`, 192.168.1.197) to the Linux box.
2. Run the extraction/analysis tooling above from `~/openpilot` on the Linux box.

## Future integration ideas

- **Ioniq 5 CAN reverse-engineering**: https://github.com/tylerharvey/Ioniq5_CAN.git — not yet
  investigated in depth, but likely applicable to the Ioniq 6 too (shared E-GMP platform/CAN
  bus) and should translate to openpilot/opendbc the same way other community DBC work has.
  Worth a look next time we're back in the Hyundai CAN weeds.
