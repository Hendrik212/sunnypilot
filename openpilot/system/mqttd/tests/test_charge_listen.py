from openpilot.system.mqttd.status import (ChargeListen, should_probe_cable, CHARGE_12V_HOLD_S, CABLE_PROBE_INTERVAL_S)


def _run(cl, t0, secs, **kw):
  on = False
  for t in range(int(secs) + 1):
    on = cl.update(t0 + t, **kw)
  return on


def test_starts_after_the_12v_hold_and_stops_below_the_off_threshold():
  cl = ChargeListen()
  kw = {"offroad": True, "awake": False, "connector_connected": None}
  assert not cl.update(0, car_voltage_filtered=14.4, **kw)
  assert _run(cl, 0, CHARGE_12V_HOLD_S, car_voltage_filtered=14.4, **kw)
  # hysteresis: 13.6 keeps it, 13.3 ends it
  assert cl.update(100, car_voltage_filtered=13.6, **kw)
  assert not cl.update(101, car_voltage_filtered=13.3, **kw)


def test_live_plugged_state_keeps_it_through_a_low_dc_dc_voltage():
  # Nov 23 13:16-14:06: charging went on at 13.0 V on the 12 V rail
  cl = ChargeListen()
  _run(cl, 0, CHARGE_12V_HOLD_S, offroad=True, awake=False, connector_connected=None, car_voltage_filtered=14.4)
  assert cl.update(100, offroad=True, awake=True, connector_connected=True, car_voltage_filtered=13.0)
  # the car went to sleep (no BMS frame for a minute): a stale plugged state does not hold the window open
  assert not cl.update(200, offroad=True, awake=False, connector_connected=True, car_voltage_filtered=12.7)


def test_unplugged_or_driving_is_off():
  cl = ChargeListen()
  assert not cl.update(0, offroad=True, awake=True, connector_connected=False, car_voltage_filtered=12.6)
  _run(cl, 0, CHARGE_12V_HOLD_S, offroad=True, awake=False, connector_connected=None, car_voltage_filtered=14.4)
  assert not cl.update(100, offroad=False, awake=True, connector_connected=True, car_voltage_filtered=14.4)


def test_no_cable_probe_during_a_charge_listen():
  args = (True, False, None, 12.6, -CABLE_PROBE_INTERVAL_S, 0.0)
  assert should_probe_cable(*args)
  assert not should_probe_cable(*args, charge_listen=True)
