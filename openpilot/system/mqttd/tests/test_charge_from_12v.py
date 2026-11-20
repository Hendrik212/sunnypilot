from openpilot.system.mqttd.status import ChargeFrom12V, CHARGE_12V_ON, CHARGE_12V_OFF, CHARGE_12V_HOLD_S


def _run(c, samples, offroad=True, awake=False):
  """samples: (t, filtered volts); returns the result of the last update."""
  out = False
  for t, v in samples:
    out = c.update(t, offroad, awake, v)
  return out


class TestChargeFrom12V:
  def test_parked_idle_car_is_not_charging(self):
    assert not _run(ChargeFrom12V(), [(t, 12.4) for t in range(0, 600, 30)])

  def test_needs_the_hold_time(self):
    c = ChargeFrom12V()
    assert not c.update(0., True, False, CHARGE_12V_ON + 0.5)
    assert not c.update(CHARGE_12V_HOLD_S - 1, True, False, CHARGE_12V_ON + 0.5)
    assert c.update(CHARGE_12V_HOLD_S, True, False, CHARGE_12V_ON + 0.5)

  def test_dip_below_on_restarts_the_hold(self):
    c = ChargeFrom12V()
    c.update(0., True, False, 14.4)
    c.update(20., True, False, 13.0)
    assert not c.update(40., True, False, 14.4)
    assert c.update(40. + CHARGE_12V_HOLD_S, True, False, 14.4)

  def test_hysteresis_keeps_it_on_between_the_thresholds(self):
    c = ChargeFrom12V()
    _run(c, [(0., 14.4), (30., 14.4)])
    assert c.active
    assert c.update(60., True, False, (CHARGE_12V_ON + CHARGE_12V_OFF) / 2)
    assert not c.update(90., True, False, CHARGE_12V_OFF - 0.01)

  def test_onroad_awake_or_unknown_voltage_resets(self):
    for kw in ({"offroad": False}, {"awake": True}):
      c = ChargeFrom12V()
      _run(c, [(0., 14.4), (30., 14.4)])
      assert c.active
      assert not _run(c, [(60., 14.4)], **kw)
      assert not c.active and c.above_since is None
    c = ChargeFrom12V()
    _run(c, [(0., 14.4), (30., 14.4)])
    assert not c.update(60., True, False, None)
