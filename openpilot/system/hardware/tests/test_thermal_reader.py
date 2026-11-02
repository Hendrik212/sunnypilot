import threading

from openpilot.common.hardware.base import ThermalConfig, ThermalZone
from openpilot.system.hardware.thermal_reader import ThermalReader


class Clock:
  def __init__(self):
    self.now = 100.

  def __call__(self):
    return self.now


class FakeZone(ThermalZone):
  def __init__(self, name, value, clock, cost=0.0, gate=None):
    super().__init__(name)
    self.value, self.clock, self.cost, self.gate = value, clock, cost, gate

  def read(self):
    if self.gate is not None:
      self.gate.wait(5.)
    self.clock.now += self.cost
    return self.value


def test_reading_has_the_get_msg_layout():
  clock = Clock()
  config = ThermalConfig(cpu=[FakeZone("cpu0", 40., clock), FakeZone("cpu1", 41., clock)],
                         pmic=[FakeZone("pm8998_tz", 35., clock)], memory=FakeZone("ddr", 38., clock))
  assert ThermalReader(config, clock=clock).latest() == {"cpuTempC": [40., 41.], "pmicTempC": [35.], "memoryTempC": 38.}


def test_a_slow_zone_is_reported_by_name():
  clock = Clock()
  slow = []
  config = ThermalConfig(cpu=[FakeZone("cpu0", 40., clock, cost=0.01)], pmic=[FakeZone("pm8998_tz", 35., clock, cost=4.4)])
  ThermalReader(config, clock=clock, on_slow=lambda z, s: slow.append((z, round(s, 2)))).read_once()
  assert slow == [("pm8998_tz", 4.4)]


def test_a_blocked_read_does_not_block_the_publisher():
  clock = Clock()
  gate = threading.Event()
  config = ThermalConfig(pmic=[FakeZone("pm8998_tz", 35., clock)])
  reader = ThermalReader(config, clock=clock)
  assert reader.latest() == {"pmicTempC": [35.]}
  config.pmic[0].gate, config.pmic[0].value = gate, 50.
  end = threading.Event()
  t = threading.Thread(target=reader.run, args=(end,), daemon=True)
  t.start()
  assert reader.latest() == {"pmicTempC": [35.]}  # reader thread is stuck in the zone read
  gate.set()
  end.set()
  t.join(2.)
  assert reader.latest() == {"pmicTempC": [50.]}
