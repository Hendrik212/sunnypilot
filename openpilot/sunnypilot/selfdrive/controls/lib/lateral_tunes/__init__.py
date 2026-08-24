"""
Lateral tune profile registry.

The selected profile is DERIVED from params on every construction, never stored: controlsd
rebuilds the lateral controller on a live tune change (check_lateral_control_version) and the
CarParams blob it holds was loaded at process start, so a stored flag could be stale.
"""
from openpilot.common.params import Params
from openpilot.sunnypilot.selfdrive.controls.lib.lateral_tunes.base import LateralTuneProfile


def get_lateral_tune_profile(CP, CP_SP, params: Params | None = None) -> LateralTuneProfile | None:
  """Return the profile for the currently selected tune, or None for the upstream path.

  Returning None (rather than a neutral profile) is deliberate: it keeps the upstream
  control path free of any profile indirection at all.
  """
  # Local imports keep the car-specific modules off the import path of every other car.
  from opendbc.sunnypilot.car.hyundai.values import is_starpilot_lat_tune

  p = params if params is not None else Params()

  if is_starpilot_lat_tune(CP, p.get("TorqueControlTune"), p.get_bool("EnforceTorqueControl")):
    from openpilot.sunnypilot.selfdrive.controls.lib.lateral_tunes.ioniq6_starpilot import Ioniq6StarPilotProfile
    return Ioniq6StarPilotProfile()

  return None
