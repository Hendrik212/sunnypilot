import openpilot.cereal.messaging as messaging
from opendbc.car.hyundai import mqtt
from openpilot.system.mqttd.status import should_probe_cable, CABLE_PROBE_INTERVAL_S, CABLE_PROBE_MIN_VOLTAGE


def _can(address, dat, src=1):
  msg = messaging.new_message('can', 1)
  msg.can[0].address = address
  msg.can[0].dat = bytes(dat)
  msg.can[0].src = src
  return msg


class TestConnectorParsing:
  def setup_method(self):
    mqtt.connector_connected_out = None

  def test_0x405_frames_from_the_wake_captures(self):
    mqtt.getParsedMessages([_can(0x405, [0xe4, 0x00, 0, 0, 0, 0, 0, 0])], 1, {})
    assert mqtt.connector_connected_out is True
    mqtt.getParsedMessages([_can(0x405, [0x8b, 0x20, 0, 0, 0, 0, 0, 0])], 1, {})
    assert mqtt.connector_connected_out is False

  def test_0x41c_checksum_byte_no_longer_decides(self):
    for dat in ([0x5e, 0x10, 0x28, 0xff, 0x03, 0, 0, 0], [0x64, 0x30, 0x28, 0, 0, 0, 0, 0]):
      mqtt.getParsedMessages([_can(0x41c, dat)], 1, {})
    assert mqtt.connector_connected_out is None

  def test_other_bus_ignored(self):
    mqtt.getParsedMessages([_can(0x405, [0x8b, 0x20, 0, 0, 0, 0, 0, 0], src=0)], 1, {})
    assert mqtt.connector_connected_out is None


class TestShouldProbeCable:
  OK = {"mqtt_connected": True, "onroad": False, "connector_connected": False, "car_voltage": 12.4, "last_probe": 0., "now": CABLE_PROBE_INTERVAL_S}

  def test_probes_when_unplugged_or_unknown(self):
    assert should_probe_cable(**self.OK)
    assert should_probe_cable(**(self.OK | {"connector_connected": None}))

  def test_never_when_plugged_in(self):
    assert not should_probe_cable(**(self.OK | {"connector_connected": True}))

  def test_needs_the_broker_and_offroad(self):
    assert not should_probe_cable(**(self.OK | {"mqtt_connected": False}))
    assert not should_probe_cable(**(self.OK | {"onroad": True}))

  def test_hourly_at_most(self):
    assert not should_probe_cable(**(self.OK | {"now": CABLE_PROBE_INTERVAL_S - 1}))

  def test_weak_or_unknown_12v_blocks(self):
    assert not should_probe_cable(**(self.OK | {"car_voltage": CABLE_PROBE_MIN_VOLTAGE - 0.01}))
    assert not should_probe_cable(**(self.OK | {"car_voltage": None}))

  def test_first_probe_right_after_start(self):
    assert should_probe_cable(**(self.OK | {"last_probe": -CABLE_PROBE_INTERVAL_S, "now": 5.}))
