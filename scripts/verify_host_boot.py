#!/usr/bin/env python3
"""Boot manager on this Linux box and require the UI process to stay alive.

Catches cereal-schema / import crashes that show up on device as a static comma
logo. Offroad only — does not exercise engaged control paths.
"""
from __future__ import annotations

import os
import sys
import time

os.environ.setdefault("NOBOARD", "1")
os.environ.setdefault("FAKEUPLOAD", "1")

HOLD_S = 8.0
TIMEOUT_S = 45.0
# Hardware / network daemons that hang or fail closed on a PC with no comma device.
NOT_RUN = [
  "pandad",
  "manage_athenad",
  "manage_sunnylinkd",
  "uploader",
  "pigeond",
  "ubloxd",
  "qcomgpsd",
]


def _schema_smoke() -> None:
  from openpilot.cereal import log

  fields = log.Event.schema.fields
  if "lateralTuneStateSP" not in fields:
    raise SystemExit("cereal Event is missing lateralTuneStateSP (static-logo class of bug)")
  ev = log.Event.new_message()
  state = ev.init("lateralTuneStateSP")
  state.profileId = "ioniq6_starpilot"
  state.active = True


def main() -> int:
  _schema_smoke()
  print("cereal Event.lateralTuneStateSP ok", flush=True)

  from opendbc.car.structs import car
  from openpilot.common.params import Params
  from openpilot.common.prefix import OpenpilotPrefix
  import openpilot.system.manager.manager as manager
  from openpilot.system.manager.process import ensure_running
  from openpilot.system.manager.process_config import managed_processes

  with OpenpilotPrefix():
    Params().put("DongleId", "UnregisteredDevice", block=True)
    manager.manager_init()
    ensure_running(managed_processes.values(), False, Params(), car.CarParams.new_message(),
                   not_run=NOT_RUN)

    ui = managed_processes["ui"]
    deadline = time.monotonic() + TIMEOUT_S
    alive_since: float | None = None
    try:
      while time.monotonic() < deadline:
        alive = ui.proc is not None and ui.proc.is_alive()
        if alive:
          if alive_since is None:
            alive_since = time.monotonic()
            print(f"ui started pid={ui.proc.pid}", flush=True)
          elif time.monotonic() - alive_since >= HOLD_S:
            print(f"ui stayed up {HOLD_S:.0f}s", flush=True)
            return 0
        elif alive_since is not None:
          code = ui.proc.exitcode if ui.proc is not None else None
          print(f"ui died after start, exitcode={code}", file=sys.stderr)
          return 1
        time.sleep(0.2)
    finally:
      manager.manager_cleanup()

    print("timeout waiting for ui to stay up", file=sys.stderr)
    if ui.proc is not None:
      print(f"ui exitcode={ui.proc.exitcode} alive={ui.proc.is_alive()}", file=sys.stderr)
    return 1


if __name__ == "__main__":
  raise SystemExit(main())
