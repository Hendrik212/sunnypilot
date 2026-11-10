from types import SimpleNamespace

import openpilot.cereal.messaging as messaging
from openpilot.system.mqttd.car_cache import CarStatusCache, LIVE_STALE_S, SAVE_INTERVAL_S
from openpilot.system.mqttd.status import live_or

WALL = 1.79e9


class FakeParams:
  def __init__(self, stored=None):
    self.stored = stored
    self.puts = 0

  def get(self, key):
    return self.stored

  def put(self, key, dat):
    self.stored = dat
    self.puts += 1


def _can(address, src=1):
  msg = messaging.new_message('can', 1)
  msg.can[0].address = address
  msg.can[0].dat = bytes(8)
  msg.can[0].src = src
  return msg


def _mqtt(**kw):
  base = {"soc_out": -1.0, "range_out": -1, "charge_limit_ac_out": -1, "charge_limit_dc_out": -1, "ac_power_limit_pct_out": -1,
          "ac_current_limit_out": -1}
  base.update(kw)
  return SimpleNamespace(**base)


class TestCarStatusCache:
  def test_restart_while_asleep_publishes_the_stored_values(self):
    params = FakeParams({"values": {"soc_out": 49.0, "range_out": 232, "charge_limit_ac_out": 80}, "ts": WALL})
    cache = CarStatusCache(params)
    m = _mqtt()  # fresh process, sleeping car: nothing read yet
    cache.update(m, [], 0.0, WALL + 3600)
    assert cache.get(m, "soc_out") == 49.0
    assert cache.get(m, "range_out") == 232
    assert cache.get(m, "charge_limit_dc_out") == -1  # never read: stays the sentinel
    assert not cache.awake(0.0)
    assert cache.timestamp().startswith("2026-")
    assert params.puts == 0  # nothing new, nothing written

  def test_live_values_win_and_are_stored(self):
    params = FakeParams({"values": {"soc_out": 49.0}, "ts": WALL})
    cache = CarStatusCache(params)
    m = _mqtt(soc_out=52.5, range_out=240)
    cache.update(m, [_can(0x2fa)], 10.0, WALL + 100)
    assert cache.awake(10.0)
    assert cache.get(m, "soc_out") == 52.5
    assert params.stored["values"]["soc_out"] == 52.5
    assert params.stored["ts"] == WALL + 100

  def test_frames_on_other_buses_or_the_wake_do_not_count_as_awake(self):
    cache = CarStatusCache(FakeParams())
    m = _mqtt(soc_out=50.0)
    cache.update(m, [_can(0x2fa, src=0), _can(0x405)], 1.0, WALL)  # the CAN wake only brings 0x405 back
    assert not cache.awake(1.0)
    assert cache.get(m, "soc_out") == 50.0  # a valid live value is still published
    assert cache.values == {}  # but only stored when the BMS is awake

  def test_asleep_after_the_stale_window(self):
    cache = CarStatusCache(FakeParams())
    cache.update(_mqtt(soc_out=50.0), [_can(0x320)], 0.0, WALL)
    assert cache.awake(LIVE_STALE_S - 1)
    assert not cache.awake(LIVE_STALE_S + 1)

  def test_unchanged_values_are_rewritten_only_every_save_interval(self):
    params = FakeParams()
    cache = CarStatusCache(params)
    m = _mqtt(soc_out=50.0)
    for t in range(0, int(SAVE_INTERVAL_S) + 10, 5):
      cache.update(m, [_can(0x2fa)], float(t), WALL + t)
    assert params.puts == 2

  def test_unsynced_wall_clock_keeps_the_old_timestamp(self):
    params = FakeParams({"values": {"soc_out": 49.0}, "ts": WALL})
    cache = CarStatusCache(params)
    cache.update(_mqtt(soc_out=50.0), [_can(0x2fa)], 1.0, 1e6)
    assert params.stored["ts"] == WALL


class TestLiveOr:
  def test_asleep_reads_as_not_charging(self):
    assert live_or(2.5, False, 0.0) == 0.0
    assert live_or(-1.0, True, 0.0) == 0.0
    assert live_or(None, True, 0.0) == 0.0
    assert live_or(2.5, True, 0.0) == 2.5
