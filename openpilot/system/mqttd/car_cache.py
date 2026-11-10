"""Last valid car values for openpilot/car_status, kept across restarts.

A parked Ioniq 6 sleeps its buses. The CAN wake brings back the body frames (0x405 plug state) but not the BMS
broadcast 0x2FA, and the comma cannot ask the BMS directly: 0x7E4 sits behind the gateway, which routes diagnostics
only from the OBD port (Nov 10 capture: requests on buses 0/1 echoed, no 0x7EC reply). So after every restart the
opendbc mqtt values start at -1 until the next drive, and publishing that -1 overwrote HA's retained values and
made it fall back to Bluelink. This keeps the last real values (Params MqttCarStatusCache) with the time they were
read, and reports a sleeping car as not charging: a charging car keeps its buses awake.
"""
import datetime

# opendbc.car.hyundai.mqtt globals that stay true while the car sleeps; -1 = never read
CACHED = ("soc_out", "range_out", "charge_limit_ac_out", "charge_limit_dc_out", "ac_power_limit_pct_out", "ac_current_limit_out")
FRESH_ADDRS = (0x2fa, 0x320)  # BMS state / charging power, bus 1: seen = the car is awake and the values are live
LIVE_STALE_S = 60.  # no BMS frame for this long = car asleep
SAVE_INTERVAL_S = 600.  # refresh the stored timestamp while the values stay the same
MIN_VALID_WALL_TIME = 1.7e9  # before NTP/GPS sync the wall clock is not usable as a data timestamp


def _valid(v):
  return v is not None and v >= 0


class CarStatusCache:
  def __init__(self, params, key="MqttCarStatusCache"):
    self.params, self.key = params, key
    stored = params.get(key) or {}
    self.values = {k: v for k, v in (stored.get("values") or {}).items() if k in CACHED and _valid(v)}
    self.ts = stored.get("ts")  # wall time of the last live BMS frame
    self.last_fresh = None  # monotonic
    self.last_save = None

  def update(self, mqtt, can_recv, now, wall):
    """Take the live values while the car is awake. now = monotonic, wall = time.time()."""
    if any(c.src == 1 and c.address in FRESH_ADDRS for m in can_recv for c in m.can):
      self.last_fresh = now
      if wall >= MIN_VALID_WALL_TIME:
        self.ts = wall
    if not self.awake(now):
      return
    live = {k: getattr(mqtt, k) for k in CACHED if _valid(getattr(mqtt, k))}
    changed = any(self.values.get(k) != v for k, v in live.items())
    self.values.update(live)
    if changed or (self.last_save is None or now - self.last_save >= SAVE_INTERVAL_S):
      self.params.put(self.key, {"values": self.values, "ts": self.ts})
      self.last_save = now

  def awake(self, now):
    return self.last_fresh is not None and now - self.last_fresh < LIVE_STALE_S

  def get(self, mqtt, name):
    """The live value if valid, else the last stored one, else the -1 sentinel."""
    v = getattr(mqtt, name)
    return v if _valid(v) else self.values.get(name, v)

  def timestamp(self):
    return datetime.datetime.fromtimestamp(self.ts, datetime.UTC).isoformat() if self.ts else None
