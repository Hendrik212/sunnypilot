"""
Lateral tune profile interface.

A profile is an optional, self-contained override of the torque controller's inner loop.
`LatControlTorque` (v2) owns the PID, the filters, the logging and the extension plumbing;
a profile owns everything that is specific to one vehicle tune. When no profile is selected
the controller runs its upstream path untouched -- `get_lateral_tune_profile()` returns None
and every call site short-circuits, so the upstream code path is byte-for-byte unchanged.

The base class below is a complete, inert profile: every attribute is the upstream-neutral
value and every hook is a no-op. A concrete profile overrides only what it needs.
"""


class LateralTuneProfile:
  # Identity. Used to decide whether controller state can be carried across a live tune
  # switch (controlsd.check_lateral_control_version). Must be unique per profile.
  profile_id = "upstream"

  # --- timing ---
  # Added to the lat_delay the controller receives. controlsd passes
  # lat_delay = lateralDelay + LAT_SMOOTH_SECONDS (modeld's constant). The shared modeld
  # now sets LAT_SMOOTH_SECONDS = 0.1, matching StarPilot, so a profile calibrated against
  # StarPilot leaves this at 0.0 -- both the lat_delay and the modeld source curvature LP
  # are reproduced at the source, with no profile-side compensation needed.
  lat_delay_offset = 0.0

  # --- gains ---
  # Denominator floor in the low-speed error boost, (interp(v) / max(v, X)) ** 2.
  low_speed_factor_min_speed = 1.0

  # --- model command shaping outside the controller ---
  # The low-speed turn-intent curvature hold (lateral_tunes/turn_intent/) is applied in
  # controlsd, upstream of clip_curvature and of the controller itself, so it cannot be a
  # method on this class. A profile opts in here; controlsd reads the flag off whichever
  # controller is live, so the hot-swap carries it too. Default off keeps the upstream
  # command path untouched.
  uses_turn_intent_hold = False

  # --- live torque parameters ---
  # torqued seeds its estimate from CP.lateralTuning.torque, i.e. from the car's
  # override.toml entry. A profile that carries its own torque baseline must not have that
  # baseline overwritten by the seed, so it declares live params off.
  use_live_torque_params = True

  def init_controller(self, ctl, CP, CP_SP, CI) -> None:
    """Allocate profile-owned state on the controller. Called at the end of __init__."""

  def filter_desired_curvature(self, ctl, CS, desired_curvature: float, active: bool) -> float:
    """Shape the model's curvature command before it reaches the controller.

    Called every frame, active or not, BEFORE the active/inactive split, so a profile that
    filters here must keep its state primed while inactive (re-engaging a time constant
    behind is a torque step). The default is the identity: upstream sees the model command
    untouched.
    """
    return desired_curvature

  def apply_live_torque_params(self, ctl, latAccelFactor, latAccelOffset, friction) -> None:
    """Write torqued's learned values onto the controller. Only called when
    use_live_torque_params is True. The default is the upstream assignment; a profile whose
    baseline is on a different normalization overrides this to convert."""
    ctl.torque_params.latAccelFactor = latAccelFactor
    ctl.torque_params.latAccelOffset = latAccelOffset
    ctl.torque_params.friction = friction

  def prime_inactive(self, ctl, CS, desired_curvature, measurement) -> None:
    """Keep profile-owned state primed while lateral is inactive."""

  def update(self, ctl, active, CS, VM, params, steer_limited_by_safety,
             desired_curvature, measured_curvature, measurement, calibrated_pose,
             pid_log, lat_delay) -> float:
    """Return output_torque. Only called when a profile is selected."""
    raise NotImplementedError
