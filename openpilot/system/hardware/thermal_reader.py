"""Thermal zone reads off the deviceState publishing thread.

A sysfs temperature read can sit in the kernel (D state, wchan thermal_zone_get_temp) for
4-8 s. hardware_thread used to read the zones inline, so deviceState stopped and selfdrived
raised commIssue (route 265, Nov 2 2026: 4.8 s and 8.1 s gaps, hardwared-watchdog.log).
ThermalReader reads in its own thread; the publisher takes the last complete reading.
Each zone read is timed and a slow one is logged by name, to find the zone that blocks.
"""
import threading
import time
from collections.abc import Callable
from dataclasses import fields

from openpilot.common.hardware.base import ThermalConfig, ThermalZone
from openpilot.common.realtime import DT_HW
from openpilot.common.swaglog import cloudlog

SLOW_READ_SECONDS = 0.1


class ThermalReader:
  def __init__(self, config: ThermalConfig, clock: Callable[[], float] = time.monotonic,
               on_slow: Callable[[str, float], None] | None = None):
    self.config = config
    self.clock = clock
    self.on_slow = on_slow if on_slow is not None else self._log_slow
    self._latest: dict | None = None
    self._latest_t: float | None = None

  @staticmethod
  def _log_slow(zone: str, seconds: float) -> None:
    cloudlog.event("thermal_zone_slow", zone=zone, seconds=round(seconds, 3), error=True)

  def _read_zone(self, zone: ThermalZone) -> float:
    t = self.clock()
    value = zone.read()
    dt = self.clock() - t
    if dt > SLOW_READ_SECONDS:
      self.on_slow(zone.name, dt)
    return value

  def read_once(self) -> dict:
    """Same layout as ThermalConfig.get_msg(), with every zone read timed."""
    ret = {}
    for f in fields(ThermalConfig):
      v = getattr(self.config, f.name)
      if v is None:
        continue
      if isinstance(v, list):
        ret[f.name + "TempC"] = [self._read_zone(x) for x in v]
      else:
        ret[f.name + "TempC"] = self._read_zone(v)
    self._latest, self._latest_t = ret, self.clock()
    return ret

  def latest(self) -> dict:
    """Last complete reading; the first call reads inline so deviceState never starts empty."""
    if self._latest is None:
      return self.read_once()
    return self._latest

  def age(self) -> float:
    return 0.0 if self._latest_t is None else self.clock() - self._latest_t

  def run(self, end_event: threading.Event) -> None:
    while not end_event.is_set():
      try:
        self.read_once()
      except Exception:
        cloudlog.exception("thermal read failed")
      end_event.wait(DT_HW)
